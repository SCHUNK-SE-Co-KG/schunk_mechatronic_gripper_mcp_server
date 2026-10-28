# Copyright 2026 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

import json
import logging
import re
import sys
from collections.abc import Callable
from dataclasses import dataclass, field

from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import EthernetScanner

from schunk_gripper_mcp_server.configuration_store import (
    load_configuration_data,
    save_configuration_data,
)
from schunk_gripper_mcp_server.common import (
    PARAMETER_ADDRESSES,
    PARAMETER_CATALOG,
    STATUS_BITS,
    decode_float_payload,
    format_gripper_status,
    validate_motion,
)

logger = logging.getLogger(__name__)

STATE_IDS = {"unconfigured": 1, "inactive": 2, "active": 3, "finalized": 4}
HEX_ADDRESS = re.compile(r"0x[0-9A-Fa-f]{1,4}")
MODBUS_UNSUPPORTED = "Modbus RTU is not supported without the ROS 2 driver. Use host and port."


@dataclass
class _Gripper:
    gripper_id: str
    host: str
    port: int
    driver: Driver = field(default_factory=Driver)
    # True while cyclic polling runs and reconnects the gripper; False after an explicit disconnect.
    reconnecting: bool = False


def _error(message: str) -> str:
    return json.dumps({"error": message})


def _result(success: bool, message: str = "") -> str:
    return json.dumps({"success": success, "message": message})


def _to_hex_payload(data: bytes | bytearray) -> str:
    return "-".join(f"{byte:02X}" for byte in data)


def _limits_reader(driver: Driver) -> Callable[[str], float | None]:
    """Read validation limits in mm or mm/s from the driver's cached module parameters (um, um/s)."""
    def read_mm(name: str) -> float | None:
        if name == "actual_pos":
            return driver.get_actual_position() / 1000.0
        value = driver.module_parameters.get(name)
        return value / 1000.0 if value is not None else None
    return read_mm


class EthernetBridge:
    """Controls grippers directly over HTTP/JSON.

    There is no lifecycle: a gripper is connected as soon as it is added or loaded, and every tool
    works at any time as long as the gripper it addresses is connected. The lifecycle tools of the
    ROS 2 driver are kept as thin helpers so the same tool calls work in both modes.
    """

    def __init__(self) -> None:
        # The library print()s, also from its threads; the MCP stdio transport already holds the real stdout.
        sys.stdout = sys.stderr
        self._grippers: list[_Gripper] = []

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _active_driver(self, gripper_id: str) -> tuple[Driver | None, str | None]:
        """Return the driver of a connected gripper, or an error message."""
        for gripper in self._grippers:
            if gripper.gripper_id == gripper_id:
                if not gripper.driver.connected:
                    if gripper.reconnecting:
                        hint = "It is retried in the background; check get_connection_state."
                    else:
                        hint = "It was disconnected explicitly. Use configure_driver to reconnect it."
                    return None, _error(f"Gripper '{gripper_id}' is not connected. {hint}")
                return gripper.driver, None
        return None, _error(f"Unknown gripper '{gripper_id}'. Use list_grippers.")

    @staticmethod
    def _connect(gripper: _Gripper) -> bool:
        """(Re)connect a gripper and acknowledge its post-boot fast stop.

        Cyclic polling keeps running even if the gripper is unreachable, so it is reconnected
        in the background once it answers.
        """
        gripper.driver.disconnect()
        gripper.driver = Driver()
        try:
            gripper.driver.connect(host=gripper.host, port=gripper.port, update_cycle=None)
        except Exception as e:
            logger.error(f"Connecting to {gripper.host}:{gripper.port} failed: {e}")
        gripper.driver.start_module_updates()
        gripper.reconnecting = True
        if gripper.driver.connected:
            try:
                gripper.driver.acknowledge()
            except Exception as e:
                logger.error(f"Acknowledge failed for {gripper.host}:{gripper.port}: {e}")
        return gripper.driver.connected

    @staticmethod
    def _command(driver: Driver, func: Callable[..., bool], *args, **kwargs) -> str:
        try:
            success = func(*args, **kwargs)
        except Exception as e:
            return _result(False, str(e))
        return _result(bool(success), driver.get_status_diagnostics())

    @staticmethod
    def _probe(host: str, port: int) -> str:
        """Return the gripper type answering at host:port, or '' if none does."""
        driver = Driver()
        try:
            return driver.gripper_type if driver.connect(host=host, port=port, update_cycle=None) else ""
        except Exception as e:
            logger.debug(f"Probing {host}:{port} failed: {e}")
            return ""
        finally:
            driver.disconnect()

    def _unique_id(self, gripper_type: str) -> str:
        """Same scheme as the ROS 2 driver: append or increment a numeric suffix."""
        known_ids = {gripper.gripper_id for gripper in self._grippers}

        def increment(name: str) -> str:
            head, _, tail = name.rpartition("_")
            return f"{head}_{int(tail) + 1}" if tail.isdigit() else f"{name}_1"

        unique_id = increment(gripper_type)
        while unique_id in known_ids:
            unique_id = increment(unique_id)
        return unique_id

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def get_state(self) -> str:
        """Derive a ROS-style state: no grippers, none connected, or at least one connected."""
        if not self._grippers:
            state = "unconfigured"
        elif any(g.driver.connected for g in self._grippers):
            state = "active"
        else:
            state = "inactive"
        return json.dumps({
            "state": state,
            "state_id": STATE_IDS[state],
            "message": "Ethernet mode has no lifecycle restrictions; every tool works at any time.",
        })

    def change_state(self, transition: str) -> str:
        handlers = {
            "configure": self._configure,
            "activate": self._activate,
            "deactivate": self._deactivate,
            "cleanup": self._disconnect_all,
            "shutdown": self._disconnect_all,
        }
        if transition not in handlers:
            return _error(f"Unknown transition: {transition}")
        return handlers[transition](transition)

    @staticmethod
    def _transition_result(transition: str, success: bool, message: str, **extra) -> str:
        return json.dumps({"success": success, "transition": transition, "message": message, **extra})

    def _connections(self) -> dict[str, bool]:
        return {g.gripper_id: g.driver.connected for g in self._grippers}

    def _configure(self, transition: str) -> str:
        """Reconnect every gripper that is not connected."""
        if not self._grippers:
            return self._transition_result(transition, False, "No grippers added yet.")
        for gripper in self._grippers:
            if not gripper.driver.connected:
                self._connect(gripper)
        connections = self._connections()
        unreachable = [gripper_id for gripper_id, connected in connections.items() if not connected]
        if unreachable:
            return self._transition_result(
                transition, False,
                f"Not connected: {', '.join(unreachable)}. They are retried in the background.",
                connections=connections,
            )
        return self._transition_result(transition, True, "All grippers are connected.", connections=connections)

    def _activate(self, transition: str) -> str:
        """Acknowledge all connected grippers. Not required before using control tools."""
        acknowledged = []
        for gripper in self._grippers:
            if not gripper.driver.connected:
                continue
            try:
                gripper.driver.acknowledge()
                acknowledged.append(gripper.gripper_id)
            except Exception as e:
                logger.error(f"Acknowledge failed for '{gripper.gripper_id}': {e}")
        if not acknowledged:
            return self._transition_result(transition, False, "No connected gripper to acknowledge.")
        return self._transition_result(transition, True, f"Acknowledged: {', '.join(acknowledged)}.")

    def _deactivate(self, transition: str) -> str:
        return self._transition_result(
            transition, True, "Nothing to do in Ethernet mode; the grippers stay connected and usable."
        )

    def _disconnect_all(self, transition: str = "cleanup") -> str:
        """Disconnect all grippers but keep them configured; configure_driver reconnects them."""
        for gripper in self._grippers:
            gripper.driver.disconnect()
            gripper.driver = Driver()
            gripper.reconnecting = False
        return self._transition_result(
            transition, True, "Disconnected all grippers. Use configure_driver to reconnect them."
        )

    # ------------------------------------------------------------------
    # Setup & discovery (available at any time)
    # ------------------------------------------------------------------

    def scan_grippers(self, scan_modbus: bool, serial_port: str) -> str:
        if scan_modbus:
            return _error(MODBUS_UNSUPPORTED)
        try:
            with EthernetScanner() as scanner:
                entries = scanner.scan()
        except OSError as e:
            return _error(f"Ethernet scan failed: {e}")
        grippers = []
        for entry in entries:
            if gripper_type := self._probe(entry["host"], entry["port"]):
                grippers.append({"type": gripper_type, "host": entry["host"], "port": entry["port"]})
        return json.dumps({"grippers": grippers})

    def add_gripper(self, host: str, port: int, serial_port: str, device_id: int) -> str:
        if serial_port or device_id:
            return _result(False, MODBUS_UNSUPPORTED)
        if not host or not 0 < port < 65536:
            return _result(False, "Provide a host and a port between 1 and 65535.")
        if any(g.host == host and g.port == port for g in self._grippers):
            return _result(False, f"A gripper at {host}:{port} has already been added.")
        gripper = _Gripper("", host, port)
        if not self._connect(gripper):
            gripper.driver.disconnect()
            return _result(False, f"No gripper answered at {host}:{port}.")
        gripper.gripper_id = self._unique_id(gripper.driver.gripper_type)
        self._grippers.append(gripper)
        return json.dumps({
            "success": True,
            "message": f"Added and connected gripper '{gripper.gripper_id}'.",
            "gripper_id": gripper.gripper_id,
        })

    def locate_gripper(self, host: str, port: int, serial_port: str, device_id: int) -> str:
        if serial_port or device_id:
            return _result(False, MODBUS_UNSUPPORTED)
        known = next(
            (g for g in self._grippers if g.host == host and g.port == port and g.driver.connected), None
        )
        driver = known.driver if known else Driver()
        try:
            if not known and not driver.connect(host=host, port=port, update_cycle=None):
                return _result(False, f"No gripper answered at {host}:{port}.")
            driver.acknowledge()
            return _result(driver.twitch_jaws())
        except Exception as e:
            return _result(False, str(e))
        finally:
            if not known:
                driver.disconnect()

    def show_configuration(self) -> str:
        return json.dumps({
            "configuration": [
                {"gripper_id": g.gripper_id, "host": g.host, "port": g.port, "connected": g.driver.connected}
                for g in self._grippers
            ]
        })

    def reset_grippers(self) -> str:
        for gripper in self._grippers:
            gripper.driver.disconnect()
        self._grippers.clear()
        return json.dumps({"success": True})

    def save_configuration(self, location: str = "") -> str:
        content = [
            {"gripper_id": g.gripper_id, "host": g.host, "port": g.port, "serial_port": "", "device_id": 0}
            for g in self._grippers
        ]
        try:
            path = save_configuration_data(content, location)
        except OSError as e:
            return _result(False, f"Cannot save configuration: {e}")
        return json.dumps({"success": True, "path": path})

    def load_previous_configuration(self, location: str = "") -> str:
        content, path, error = load_configuration_data(location)
        if error:
            return _result(False, error)

        grippers = []
        for entry in content:
            if not isinstance(entry, dict):
                return _result(False, f"Unexpected layout in {path}.")
            if entry.get("serial_port"):
                return _result(False, f"Saved gripper '{entry.get('gripper_id')}' uses Modbus RTU. {MODBUS_UNSUPPORTED}")
            gripper_id, host, port = entry.get("gripper_id"), entry.get("host"), entry.get("port")
            if not (gripper_id and isinstance(gripper_id, str) and host and isinstance(host, str) and isinstance(port, int)):
                return _result(False, f"Invalid entry in {path}: {entry}")
            grippers.append(_Gripper(gripper_id, host, port))
        for gripper in self._grippers:
            gripper.driver.disconnect()
        self._grippers = grippers
        for gripper in grippers:
            self._connect(gripper)
        return json.dumps({
            "success": True,
            "grippers": [g.gripper_id for g in grippers],
            "connections": self._connections(),
        })

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------

    def list_grippers(self) -> str:
        return json.dumps({"grippers": [g.gripper_id for g in self._grippers]})

    # ------------------------------------------------------------------
    # Gripper control (requires a connected gripper)
    # ------------------------------------------------------------------

    def _move(self, gripper_id: str, position: float, velocity: float, use_gpe: bool, is_absolute: bool) -> str:
        driver, error = self._active_driver(gripper_id)
        if error:
            return error
        if message := validate_motion(
            gripper_id, _limits_reader(driver),
            position=position, relative_position=not is_absolute, velocity=velocity,
        ):
            return _result(False, message)
        return self._command(
            driver, driver.move_to_position,
            position=round(position * 1e6), velocity=round(velocity * 1e6),
            is_absolute=is_absolute, use_gpe=use_gpe,
        )

    def move_to_absolute_position(self, gripper_id: str, position: float, velocity: float, use_gpe: bool) -> str:
        return self._move(gripper_id, position, velocity, use_gpe, is_absolute=True)

    def move_to_relative_position(self, gripper_id: str, position: float, velocity: float, use_gpe: bool) -> str:
        return self._move(gripper_id, position, velocity, use_gpe, is_absolute=False)

    def grip(
        self,
        gripper_id: str,
        force: int,
        outward: bool,
        position: float | None,
        velocity: float | None,
        use_gpe: bool,
    ) -> str:
        driver, error = self._active_driver(gripper_id)
        if error:
            return error
        if message := validate_motion(
            gripper_id, _limits_reader(driver),
            position=position, velocity=velocity, force=force, grip=True,
        ):
            return _result(False, message)
        if velocity is not None and driver.get_variant() != "EGK":
            return _result(False, f"Gripper '{gripper_id}' does not support a gripping velocity (EGK SoftGrip only).")
        # StrongGrip always engages GPE, and Driver.grip() waits for status bit 31 == use_gpe.
        use_gpe = driver.gpe_available() and (use_gpe or force > 100)
        try:
            result = driver.grip(
                force=force,
                position=None if position is None else round(position * 1e6),
                velocity=None if velocity is None else round(velocity * 1e6),
                use_gpe=use_gpe,
                outward=outward,
            )
        except Exception as e:
            return _result(False, str(e))
        return json.dumps({
            "success": result != Driver.GripResult.ERROR,
            "message": driver.get_status_diagnostics(),
            "workpiece_gripped": result == Driver.GripResult.WORKPIECE_GRIPPED,
            "no_workpiece_detected": result == Driver.GripResult.NO_WORKPIECE_DETECTED,
            "wrong_workpiece_gripped": result == Driver.GripResult.WRONG_WORKPIECE_GRIPPED,
            "workpiece_lost": result == Driver.GripResult.WORKPIECE_LOST,
        })

    def release(self, gripper_id: str, use_gpe: bool) -> str:
        driver, error = self._active_driver(gripper_id)
        return error or self._command(driver, driver.release, use_gpe=use_gpe)

    def acknowledge(self, gripper_id: str) -> str:
        driver, error = self._active_driver(gripper_id)
        return error or self._command(driver, driver.acknowledge)

    def stop(self, gripper_id: str, use_gpe: bool) -> str:
        driver, error = self._active_driver(gripper_id)
        return error or self._command(driver, driver.stop, use_gpe=use_gpe)

    def fast_stop(self, gripper_id: str) -> str:
        driver, error = self._active_driver(gripper_id)
        return error or self._command(driver, driver.fast_stop)

    def start_jogging(self, gripper_id: str, velocity: float, use_gpe: bool) -> str:
        driver, error = self._active_driver(gripper_id)
        if error:
            return error
        if message := validate_motion(gripper_id, _limits_reader(driver), velocity=abs(velocity)):
            return _result(False, message)
        return self._command(driver, driver.start_jogging, velocity=round(velocity * 1e6), use_gpe=use_gpe)

    def stop_jogging(self, gripper_id: str) -> str:
        driver, error = self._active_driver(gripper_id)
        return error or self._command(driver, driver.stop_jogging)

    def prepare_for_shutdown(self, gripper_id: str) -> str:
        driver, error = self._active_driver(gripper_id)
        return error or self._command(driver, driver.prepare_for_shutdown)

    def soft_reset(self, gripper_id: str) -> str:
        driver, error = self._active_driver(gripper_id)
        return error or self._command(driver, driver.soft_reset)

    def brake_test(self, gripper_id: str) -> str:
        driver, error = self._active_driver(gripper_id)
        if error:
            return error
        if not driver.gpe_available():
            return _error(f"Gripper '{gripper_id}' has no GPE/brake, so brake_test is not available.")
        return self._command(driver, driver.brake_test)

    # ------------------------------------------------------------------
    # Status & diagnostics (requires a connected gripper)
    # ------------------------------------------------------------------

    def get_gripper_status(self, gripper_id: str) -> str:
        driver, error = self._active_driver(gripper_id)
        if error:
            return error
        error_code, warning_code, additional_code = (
            part.strip() for part in driver.get_status_diagnostics().split(",")
        )
        status = {name: bool(driver.get_status_bit(bit)) for bit, name in STATUS_BITS.items()}
        return format_gripper_status(gripper_id, error_code, warning_code, additional_code, status)

    def get_gripper_position(self, gripper_id: str) -> str:
        driver, error = self._active_driver(gripper_id)
        if error:
            return error
        return json.dumps({"gripper_id": gripper_id, "position_m": driver.get_actual_position() / 1e6})

    def get_connection_state(self) -> str:
        return json.dumps({"connections": {g.gripper_id: g.driver.connected for g in self._grippers}})

    def get_gripper_specification(self, gripper_id: str) -> str:
        driver, error = self._active_driver(gripper_id)
        if error:
            return error
        try:
            spec = driver.show_specification()
        except Exception as e:
            return _result(False, str(e))
        return json.dumps({
            "success": True,
            "gripper_id": gripper_id,
            "max_stroke_mm": spec["max_stroke"],
            "max_speed_mm_s": spec["max_speed"],
            "max_force_n": spec["max_force"],
            "serial_number": spec["serial_number"],
            "firmware_version": spec["firmware_version"],
            "device_id": spec["device_id"],
            "ip_address": spec["ip_address"],
        })

    def list_parameters(self) -> str:
        return json.dumps({"parameters": PARAMETER_CATALOG})

    def _resolve_parameter(self, parameter: str, length: int) -> tuple[str, int, str | None]:
        code = PARAMETER_ADDRESSES.get(parameter, parameter)
        if not HEX_ADDRESS.fullmatch(code):
            return code, length, _error(
                f"Unknown parameter '{parameter}'. Use a name from list_gripper_parameters or a hex address like '0x0650'."
            )
        if length == 0:
            length = PARAMETER_CATALOG.get(parameter, {}).get("registers", 0)
        return code, length, None

    def read_parameter_raw(self, gripper_id: str, parameter: str, length: int = 0) -> str:
        driver, error = self._active_driver(gripper_id)
        if error:
            return error
        code, length, error = self._resolve_parameter(parameter, length)
        if error:
            return error
        try:
            data = driver.read_param(code, read_raw=True, length=length)
        except Exception as e:
            return json.dumps({"success": False, "gripper_id": gripper_id, "parameter": code, "message": str(e)})
        if not data:
            return json.dumps({"success": False, "gripper_id": gripper_id, "parameter": code, "message": "Failed to read parameter. Check the parameter address and length."})
        result = {"success": True, "gripper_id": gripper_id, "parameter": code, "payload": _to_hex_payload(data)}
        # PROFINET grippers transmit parameter values big-endian.
        decoded_float = decode_float_payload(_to_hex_payload(data[::-1] if driver.fieldbus == "PN" else data))
        if decoded_float is not None:
            result["decoded_float"] = decoded_float
        return json.dumps(result)

    def write_parameter_raw(self, gripper_id: str, parameter: str, payload: str, length: int = 0) -> str:
        driver, error = self._active_driver(gripper_id)
        if error:
            return error
        code, length, error = self._resolve_parameter(parameter, length)
        if error:
            return error
        try:
            data = bytearray(int(byte, 16) for byte in payload.split("-"))
        except ValueError:
            return _error("Payload must be hyphen-separated hex bytes, e.g. '42-04-00-00'.")
        try:
            success = driver.write_param(code, data, write_raw=True, length=length)
        except Exception as e:
            return json.dumps({"success": False, "gripper_id": gripper_id, "parameter": code, "message": str(e)})
        return json.dumps({"success": success, "gripper_id": gripper_id, "parameter": code})
