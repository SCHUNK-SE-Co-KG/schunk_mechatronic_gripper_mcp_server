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

from pathlib import Path

from mcp.server.fastmcp import FastMCP
from mcp.types import TextResourceContents
from schunk_gripper_mcp_server.ros_bridge import RosBridge

INSTRUCTIONS = """\
You are controlling SCHUNK EGU / EGK / EZU mechatronic grippers through a ROS 2 driver.

Gripper variants:
- EGU: 2-finger parallel gripper, strong-grip
- EGK: 2-finger parallel gripper, soft-grip (sensitive)
- EZU: 3-finger centric gripper, strong-grip
Module suffixes: M = with GPE (grip force & position maintenance), N = without GPE.
All variants share the same command set and status model.

Positions are total jaw-to-jaw positions in meters. Force is a percentage of max_grp_force:
- BasicGrip (all variants): 50-100%.
- SoftGrip (EGK only): 50-100%, with a gripping velocity between min_vel and force%*max_grp_vel.
- StrongGrip (EGU/EZU M modules only): 101-200% (EGU 50/60/80, EZU), 101-150% (EGU 70). GPE activates
  automatically. Never available on EGK or on N modules — out-of-range/wrong-mode force is rejected
  (WRN_NOT_FEASIBLE, additional code 0x29).

Max stroke per size (mm): EGU 50→102, 60→120, 70→140, 80→160;
EGK 25→53, 40→83, 50→103; EZU 30→60, 35→70, 40→80.
Use `get_gripper_specification` to get the exact limits for a connected gripper.
Use `list_gripper_parameters` to look up the full catalog of documented Modbus module
parameters (diagnostics, force/velocity limits, voltages, temperatures, device info,
Modbus RTU settings) before using `read_gripper_parameter_raw` / `write_gripper_parameter_raw`.

Key status conditions to always check:
- error: gripper left operation, forced to standstill. Use `acknowledge` to clear (once cause is resolved).
- workpiece_lost: workpiece was lost during hold — act immediately.
- wrong_workpiece_gripped: gripped workpiece size outside expected window.
- no_workpiece_detected: grip found no workpiece.
- not_feasible: last command was rejected (see diagnosis codes for reason).
- warning: non-fatal condition present (see diagnosis codes).
Never ignore error, workpiece_lost, wrong_workpiece_gripped, or no_workpiece_detected.

GPE (grip force & position maintenance) is only available on M modules. Never request GPE on N modules.

Modbus RTU transport notes (when a gripper was added via serial_port/device_id):
- RS-485, 8E1, default baud rate 115200, default slave/device ID 12 (1-247 valid). Function codes
  0x03 (read parameters) and 0x10 (write parameters) only.
- Register addresses equal the parameter's hex code (e.g. plc_sync_output = 0x0048). Every register is
  16 bits; multi-byte parameters (floats, int32) span 2+ consecutive registers and must be read/written
  as a whole — never partially.
- Parameter payload values (position, velocity, force, control/status word) are little-endian. Only the
  raw Modbus protocol fields (register address, quantity) are big-endian — don't confuse the two when
  using `read_parameter_raw` / `write_parameter_raw`.
- If communication is interrupted (e.g. cable break), the gripper performs a fast stop and reports
  ERR_COMM_LOST — reconnect and `acknowledge` before continuing.

The driver is a lifecycle node. Before you can control grippers, you must set it up:

1. Check the driver state with `get_driver_state`.
2. If the state is UNCONFIGURED and no grippers are configured yet:
   a. Use `scan_grippers` to discover grippers on the network (Ethernet) or serial bus (Modbus RTU).
   b. Use `add_gripper` for each gripper you want to control.
      - For Ethernet grippers: provide host and port.
      - For Modbus grippers: provide serial_port and device_id.
   c. Optionally use `locate_gripper` to physically identify a gripper (it twitches its jaws).
   d. Use `save_configuration` to persist the setup so it can be reloaded on next start.
   e. Or use `load_previous_configuration` to restore a previously saved setup.
3. Transition to INACTIVE with `configure_driver` (this connects to all added grippers).
4. Transition to ACTIVE with `activate_driver` (this enables gripper control).
5. Now you can use gripper control and status tools.

Typical pick workflow:
1. Acknowledge any errors.
2. Move to pre-grip position (move_to_absolute_position).
3. Grip workpiece (grip) — check workpiece_gripped, no_workpiece_detected, wrong_workpiece_gripped.
4. Move to place position.
5. Release workpiece.

If a command fails, check the driver state — it may be in the wrong lifecycle state for that operation.
Use `get_driver_state` to diagnose. Setup tools only work in UNCONFIGURED state.
Control tools only work in ACTIVE state.

`get_gripper_status` returns human-readable descriptions for all diagnosis codes.
"""

_MEMORY_PATH = Path(__file__).with_name("agent_memory.md")
if _MEMORY_PATH.is_file():
    INSTRUCTIONS = _MEMORY_PATH.read_text(encoding="utf-8")

mcp = FastMCP(
    "Schunk Gripper MCP Server",
    instructions=INSTRUCTIONS,
)


# ---------------------------------------------------------------------------
# Resource: Gripper Firmware Documentation
# ---------------------------------------------------------------------------

@mcp.resource("gripper://documentation/schunk-firmware-5.3.1")
def get_gripper_documentation() -> TextResourceContents:
    """SCHUNK EGU/EGK/EZU Gripper Firmware Reference (5.3.1).

    Comprehensive documentation covering:
    - Gripper variants and their specifications
    - Gripping modes and force ranges
    - Control words and status bits
    - Command catalog and workflows
    - Cyclic/acyclic parameters
    - Modbus RTU communication details
    - Diagnosis codes and error handling
    - Safety constraints and best practices

    Use this resource to understand gripper capabilities, parameter requirements,
    and proper control sequences.
    """
    memory_path = Path(__file__).with_name("agent_memory.md")
    if memory_path.is_file():
        return TextResourceContents(uri="gripper://documentation/schunk-firmware-5.3.1", text=memory_path.read_text(encoding="utf-8"))
    else:
        # Fallback if agent_memory.md doesn't exist
        return TextResourceContents(
            uri="gripper://documentation/schunk-firmware-5.3.1",
            text="SCHUNK Gripper documentation not available. Please ensure agent_memory.md is present."
        )


bridge: RosBridge | None = None


def get_bridge() -> RosBridge:
    global bridge
    if bridge is None:
        bridge = RosBridge()
    return bridge


# ---------------------------------------------------------------------------
# Lifecycle management tools
# ---------------------------------------------------------------------------


@mcp.tool()
async def get_driver_state() -> str:
    """Get the current lifecycle state of the gripper driver node.

    Returns the state name: 'unconfigured', 'inactive', 'active',
    'finalized', or 'unknown'.
    """
    return get_bridge().get_state()


@mcp.tool()
async def configure_driver() -> str:
    """Transition the driver from UNCONFIGURED to INACTIVE.

    This connects to all added grippers. Requires at least one gripper
    to be added first via add_gripper or load_previous_configuration.
    """
    return get_bridge().change_state("configure")


@mcp.tool()
async def activate_driver() -> str:
    """Transition the driver from INACTIVE to ACTIVE.

    This enables gripper control services (move, grip, release, etc.).
    """
    return get_bridge().change_state("activate")


@mcp.tool()
async def deactivate_driver() -> str:
    """Transition the driver from ACTIVE to INACTIVE.

    This disables gripper control but keeps connections alive.
    """
    return get_bridge().change_state("deactivate")


@mcp.tool()
async def cleanup_driver() -> str:
    """Transition the driver from INACTIVE to UNCONFIGURED.

    If the driver is ACTIVE, deactivate it first. This then disconnects from
    all grippers and re-enables setup services.
    """
    return get_bridge().change_state("cleanup")


@mcp.tool()
async def shutdown_driver() -> str:
    """Shutdown the driver node completely."""
    return get_bridge().change_state("shutdown")


# ---------------------------------------------------------------------------
# Setup & discovery tools (unconfigured state)
# ---------------------------------------------------------------------------


@mcp.tool()
async def scan_grippers(scan_modbus: bool = False, serial_port: str = "/dev/ttyUSB0") -> str:
    """Scan for available grippers on the network or serial bus.

    Args:
        scan_modbus: If True, scan Modbus RTU serial bus. If False, scan Ethernet.
        serial_port: Serial port to use for Modbus scanning (default: /dev/ttyUSB0).

    Returns a list of discovered grippers with their connection details.
    Only available in UNCONFIGURED state.
    """
    return get_bridge().scan_grippers(scan_modbus=scan_modbus, serial_port=serial_port)


@mcp.tool()
async def add_gripper(
    host: str = "",
    port: int = 0,
    serial_port: str = "",
    device_id: int = 0,
) -> str:
    """Add a gripper to the driver configuration.

    For Ethernet grippers, provide host and port.
    For Modbus RTU grippers, provide serial_port and device_id.

    Args:
        host: TCP/IP host address (e.g. "192.168.1.10").
        port: TCP/IP port (e.g. 80).
        serial_port: Serial port path (e.g. "/dev/ttyUSB0").
        device_id: Modbus device ID (e.g. 12).

    Only available in UNCONFIGURED state.
    """
    return get_bridge().add_gripper(
        host=host, port=port, serial_port=serial_port, device_id=device_id
    )


@mcp.tool()
async def locate_gripper(
    host: str = "",
    port: int = 0,
    serial_port: str = "",
    device_id: int = 0,
) -> str:
    """Physically identify a gripper by twitching its jaws.

    Provide connection details to locate a specific gripper.
    Only available in UNCONFIGURED state.
    """
    return get_bridge().locate_gripper(
        host=host, port=port, serial_port=serial_port, device_id=device_id
    )


@mcp.tool()
async def show_configuration() -> str:
    """Show the current gripper configuration (all added grippers and their connection details).

    Only available in UNCONFIGURED state.
    """
    return get_bridge().show_configuration()


@mcp.tool()
async def reset_grippers() -> str:
    """Remove all grippers from the current configuration.

    Only available in UNCONFIGURED state.
    """
    return get_bridge().reset_grippers()


@mcp.tool()
async def save_configuration() -> str:
    """Save the current gripper configuration to disk.

    The configuration is persisted at /var/tmp/schunk_gripper/configuration.json
    and will be available for load_previous_configuration on the next start.
    Only available in UNCONFIGURED state.
    """
    return get_bridge().save_configuration()


@mcp.tool()
async def load_previous_configuration() -> str:
    """Load a previously saved gripper configuration from disk.

    Restores grippers from /var/tmp/schunk_gripper/configuration.json.
    Only available in UNCONFIGURED state.
    """
    return get_bridge().load_previous_configuration()


# ---------------------------------------------------------------------------
# Info tools (configured state and above)
# ---------------------------------------------------------------------------


@mcp.tool()
async def list_grippers() -> str:
    """List all configured gripper IDs.

    Available in INACTIVE and ACTIVE states (after configure_driver).
    Returns a list of gripper IDs that can be used with other tools.
    """
    return get_bridge().list_grippers()


# ---------------------------------------------------------------------------
# Gripper control tools (active state)
# ---------------------------------------------------------------------------


@mcp.tool()
async def move_to_absolute_position(
    gripper_id: str,
    position: float,
    velocity: float,
    use_gpe: bool = False,
) -> str:
    """Move the gripper to an absolute total jaw-to-jaw position.

    Args:
        gripper_id: The gripper to control (from list_grippers).
        position: Target total jaw-to-jaw position in meters.
        velocity: Movement velocity in m/s.
        use_gpe: Activate grip force and position maintenance after moving.

    Only available in ACTIVE state.
    """
    return get_bridge().move_to_absolute_position(
        gripper_id=gripper_id, position=position, velocity=velocity, use_gpe=use_gpe
    )


@mcp.tool()
async def move_to_relative_position(
    gripper_id: str,
    position: float,
    velocity: float,
    use_gpe: bool = False,
) -> str:
    """Move the gripper by a relative total jaw-to-jaw offset.

    Args:
        gripper_id: The gripper to control (from list_grippers).
        position: Relative total jaw-to-jaw offset in meters (positive = open, negative = close).
        velocity: Movement velocity in m/s.
        use_gpe: Activate grip force and position maintenance after moving.

    Only available in ACTIVE state.
    """
    return get_bridge().move_to_relative_position(
        gripper_id=gripper_id, position=position, velocity=velocity, use_gpe=use_gpe
    )


@mcp.tool()
async def grip(
    gripper_id: str,
    force: int,
    outward: bool = False,
    velocity: float | None = None,
    use_gpe: bool = False,
) -> str:
    """Close the gripper with specified force.

    Args:
        gripper_id: The gripper to control (from list_grippers).
        force: Grip force in percent of max_grp_force. 50-100 = BasicGrip/SoftGrip (all variants).
            101-200 (EGU 50/60/80, EZU) or 101-150 (EGU 70) = StrongGrip, only on EGU/EZU M (GPE)
            modules — never on EGK or N modules; GPE activates automatically for StrongGrip.
        outward: If True, grip outward (fingers moving apart). Default: False (inward grip).
        velocity: Optional gripping velocity in m/s (EGK SoftGrip only; 0 = BasicGrip speed derived
            from force). EGU/EZU ignore this field.
        use_gpe: Activate grip force and position maintenance.

    Only available in ACTIVE state.
    """
    return get_bridge().grip(
        gripper_id=gripper_id,
        force=force,
        outward=outward,
        position=None,
        velocity=velocity,
        use_gpe=use_gpe,
    )


@mcp.tool()
async def grip_at_position(
    gripper_id: str,
    force: int,
    position: float,
    outward: bool = False,
    velocity: float | None = None,
    use_gpe: bool = False,
) -> str:
    """Grip a workpiece at an expected jaw-to-jaw position.

    Args:
        gripper_id: The gripper to control (from list_grippers).
        force: Grip force in percent of max_grp_force.
        position: Expected total jaw-to-jaw workpiece position in meters.
        outward: If True, grip outward (fingers moving apart). Default: False.
        velocity: Optional gripping velocity in m/s (EGK SoftGrip only).
        use_gpe: Activate grip force and position maintenance.

    Only available in ACTIVE state.
    """
    return get_bridge().grip(
        gripper_id=gripper_id,
        force=force,
        outward=outward,
        position=position,
        velocity=velocity,
        use_gpe=use_gpe,
    )


@mcp.tool()
async def release(gripper_id: str, use_gpe: bool = False) -> str:
    """Release/open the gripper.

    Args:
        gripper_id: The gripper to control (from list_grippers).
        use_gpe: Use position maintenance after releasing.

    Only available in ACTIVE state.
    """
    return get_bridge().release(gripper_id=gripper_id, use_gpe=use_gpe)


@mcp.tool()
async def acknowledge(gripper_id: str) -> str:
    """Acknowledge gripper errors. Call this to clear error states.

    Args:
        gripper_id: The gripper to acknowledge (from list_grippers).

    Only available in ACTIVE state.
    """
    return get_bridge().acknowledge(gripper_id=gripper_id)


@mcp.tool()
async def stop(gripper_id: str, use_gpe: bool = False) -> str:
    """Stop the current gripper motion.

    Args:
        gripper_id: The gripper to stop (from list_grippers).
        use_gpe: Use position maintenance after stopping.

    Only available in ACTIVE state.
    """
    return get_bridge().stop(gripper_id=gripper_id, use_gpe=use_gpe)


@mcp.tool()
async def fast_stop(gripper_id: str) -> str:
    """Emergency stop the gripper. Requires acknowledge before further commands.

    Args:
        gripper_id: The gripper to stop (from list_grippers).

    Only available in ACTIVE state.
    """
    return get_bridge().fast_stop(gripper_id=gripper_id)


@mcp.tool()
async def start_jogging(
    gripper_id: str, velocity: float, use_gpe: bool = False
) -> str:
    """Start continuous jogging motion.

    Args:
        gripper_id: The gripper to control (from list_grippers).
        velocity: Jogging velocity in m/s (positive = open, negative = close).
        use_gpe: Activate position maintenance after stopping.

    Only available in ACTIVE state.
    """
    return get_bridge().start_jogging(
        gripper_id=gripper_id, velocity=velocity, use_gpe=use_gpe
    )


@mcp.tool()
async def stop_jogging(gripper_id: str) -> str:
    """Stop jogging motion.

    Args:
        gripper_id: The gripper to control (from list_grippers).

    Only available in ACTIVE state.
    """
    return get_bridge().stop_jogging(gripper_id=gripper_id)


@mcp.tool()
async def prepare_for_shutdown(gripper_id: str) -> str:
    """Prepare the gripper for safe power-off.

    Args:
        gripper_id: The gripper to prepare (from list_grippers).

    Only available in ACTIVE state.
    """
    return get_bridge().prepare_for_shutdown(gripper_id=gripper_id)


@mcp.tool()
async def soft_reset(gripper_id: str) -> str:
    """Soft-reset the gripper module.

    Args:
        gripper_id: The gripper to reset (from list_grippers).

    Only available in ACTIVE state.
    """
    return get_bridge().soft_reset(gripper_id=gripper_id)


@mcp.tool()
async def brake_test(gripper_id: str) -> str:
    """Run a brake test on a gripper.

    Calls the gripper-specific ROS Trigger service at
    /schunk/driver/<gripper_id>/brake_test.

    Args:
        gripper_id: The gripper to test (from list_grippers).

    Only available in ACTIVE state and only on grippers that expose brake_test.
    """
    return get_bridge().brake_test(gripper_id=gripper_id)


# ---------------------------------------------------------------------------
# Status & diagnostics tools (active state)
# ---------------------------------------------------------------------------


@mcp.tool()
async def get_gripper_status(gripper_id: str) -> str:
    """Get the full status of a specific gripper.

    Returns diagnostic codes with human-readable descriptions and all status bits:
    ready_for_operation, workpiece_gripped, position_reached, error, warning,
    workpiece_lost, wrong_workpiece_gripped, no_workpiece_detected, not_feasible, etc.
    Active conditions are summarized in the 'active_conditions' list.

    Args:
        gripper_id: The gripper to query (from list_grippers).

    Only available in ACTIVE state.
    """
    return get_bridge().get_gripper_status(gripper_id=gripper_id)


@mcp.tool()
async def get_gripper_position(gripper_id: str) -> str:
    """Get the current jaw position of a gripper.

    Args:
        gripper_id: The gripper to query (from list_grippers).

    Only available in ACTIVE state.
    """
    return get_bridge().get_gripper_position(gripper_id=gripper_id)


@mcp.tool()
async def get_connection_state() -> str:
    """Get the connection state of all configured grippers.

    Returns which grippers are connected and which are disconnected.
    Available whenever the driver is running.
    """
    return get_bridge().get_connection_state()


@mcp.tool()
async def get_gripper_specification(gripper_id: str) -> str:
    """Get hardware specifications of a gripper.

    Returns max stroke, max speed, max force, serial number, firmware version,
    and connection info.

    Args:
        gripper_id: The gripper to query (from list_grippers).

    Only available in ACTIVE state.
    """
    return get_bridge().get_gripper_specification(gripper_id=gripper_id)


@mcp.tool()
async def list_gripper_parameters() -> str:
    """List all known Modbus module parameters (name, hex code, type, unit, access, register count).

    Use this to look up the hex code and register count for a parameter name before
    calling read_gripper_parameter_raw / write_gripper_parameter_raw. Covers the full
    parameter set documented in the EGK/EGU Modbus RTU commissioning manuals
    (firmware 5.3.1), including diagnostics, force/velocity limits, voltage/temperature
    thresholds, device identification and Modbus RTU settings (baudrate, slave_id).
    """
    return get_bridge().list_parameters()


@mcp.tool()
async def read_gripper_parameter_raw(gripper_id: str, parameter: str, length: int = 0) -> str:
    """Read a raw IO-Link module parameter via the driver's hidden _read_parameter_raw service.

    Accepts either a known parameter name (see list_gripper_parameters, e.g. "max_grp_vel")
    or a raw hex address (e.g. "0x0650").

    Args:
        gripper_id: The gripper to query (from list_grippers).
        parameter: Parameter name or module address in hex, e.g. "max_grp_vel" or "0x0650".
        length: Number of registers to read (0 = use the parameter's known register count).

    Returns the payload as a hyphenated hex string (e.g. "42-04-00-00"), plus a
    best-effort 'decoded_float' if the payload is 4 bytes (most numeric parameters).

    Only available in ACTIVE state.
    """
    return get_bridge().read_parameter_raw(gripper_id=gripper_id, parameter=parameter, length=length)


@mcp.tool()
async def write_gripper_parameter_raw(gripper_id: str, parameter: str, payload: str, length: int = 0) -> str:
    """Write a raw IO-Link module parameter via the driver's hidden _write_parameter_raw service.

    Accepts either a known parameter name (see list_gripper_parameters, e.g. "grp_prehold_time")
    or a raw hex address (e.g. "0x0380").

    Caution: writing incorrect values to module parameters can put the gripper
    into an unsafe or unusable state. Only write parameters you understand.

    Args:
        gripper_id: The gripper to write to (from list_grippers).
        parameter: Parameter name or module address in hex, e.g. "grp_prehold_time" or "0x0380".
        payload: Byte values as a hyphenated hex string, e.g. "42-04-00-00".
        length: Number of registers being written (0 = use the parameter's known register count).

    Only available in ACTIVE state.
    """
    return get_bridge().write_parameter_raw(
        gripper_id=gripper_id, parameter=parameter, payload=payload, length=length
    )


def main():
    mcp.run()


if __name__ == "__main__":
    main()
