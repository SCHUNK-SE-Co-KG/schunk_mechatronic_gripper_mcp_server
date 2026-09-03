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
import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from std_srvs.srv import Trigger
from schunk_gripper_interfaces.srv import (
    ListGrippers,
    AddGripper,
    ScanGrippers,
    LocateGripper,
    ShowConfiguration,
    ShowGripperSpecification,
    MoveToAbsolutePosition,
    MoveToAbsolutePositionGPE,
    MoveToRelativePosition,
    MoveToRelativePositionGPE,
    Grip,
    GripWithGPE,
    GripWithVelocity,
    GripWithVelocityAndGPE,
    GripAtPosition,
    GripAtPositionWithGPE,
    GripAtPositionWithVelocity,
    GripAtPositionWithVelocityAndGPE,
    Release,
    ReleaseWithGPE,
    StartJogging,
    StartJoggingGPE,
    Stop,
    StopWithGPE,
    ReadGripperParameterRaw,
    WriteGripperParameterRaw,
)
from schunk_gripper_interfaces.msg import GripperState, ConnectionState
from sensor_msgs.msg import JointState


# Map lifecycle state IDs to names
STATE_NAMES = {
    0: "unknown",
    1: "unconfigured",
    2: "inactive",
    3: "active",
    4: "finalized",
}

# Map transition names to IDs
TRANSITIONS = {
    "configure": Transition.TRANSITION_CONFIGURE,
    "activate": Transition.TRANSITION_ACTIVATE,
    "deactivate": Transition.TRANSITION_DEACTIVATE,
    "cleanup": Transition.TRANSITION_CLEANUP,
    "shutdown": Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
}

DRIVER_NODE = "/schunk/driver"
SERVICE_TIMEOUT = 20.0

# Configure logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

PARAMETER_ADDRESSES = {
    "actual_pos": "0x0230",
    "min_pos": "0x0600",
    "max_pos": "0x0608",
    "min_vel": "0x0628",
    "max_vel": "0x0630",
    "max_grp_vel": "0x0650",
}

# Full parameter catalog from the EGK/EGU Modbus RTU commissioning manuals
# (firmware 5.3.1, §4.2). "registers" = number of 16-bit Modbus registers the
# parameter occupies; "access" = ro (read-only) / rw (read+write).
PARAMETER_CATALOG = {
    "plc_sync_input": {"code": "0x0040", "type": "struct", "unit": None, "access": "ro", "registers": 8, "note": "Cyclic status word + actual position + diagnosis word"},
    "plc_sync_output": {"code": "0x0048", "type": "struct", "unit": None, "access": "rw", "registers": 8, "note": "Cyclic control word + target position/velocity/force"},
    "err_code": {"code": "0x0118", "type": "enum", "unit": None, "access": "ro", "registers": 1, "note": "Current error code"},
    "wrn_code": {"code": "0x0120", "type": "enum", "unit": None, "access": "ro", "registers": 1, "note": "Current warning code"},
    "sys_msg_req": {"code": "0x0128", "type": "uint16", "unit": None, "access": "rw", "registers": 1, "note": "Write diagnostic-memory index (0-31) to select entry for sys_msg_buffer"},
    "sys_msg_buffer": {"code": "0x0130", "type": "char[214]", "unit": None, "access": "ro", "registers": 107, "note": "ASCII diagnostic-memory entry selected via sys_msg_req"},
    "actual_pos": {"code": "0x0230", "type": "float", "unit": "mm", "access": "ro", "registers": 2, "note": None},
    "actual_vel": {"code": "0x0238", "type": "float", "unit": "mm/s", "access": "ro", "registers": 2, "note": None},
    "grp_prehold_time": {"code": "0x0380", "type": "uint16", "unit": "ms", "access": "rw", "registers": 1, "note": "Re-gripping time; range 0-5000, default 0 (0 = no re-gripping)"},
    "dead_load_kg": {"code": "0x03A8", "type": "float", "unit": "kg", "access": "ro", "registers": 2, "note": "Net mass of the gripper"},
    "tool_cent_point": {"code": "0x03B0", "type": "float[6]", "unit": "mm/deg", "access": "ro", "registers": 12, "note": "TCP 6D frame: x,y,z [mm], a,b,c [deg]"},
    "cent_of_mass": {"code": "0x03B8", "type": "float[6]", "unit": "mm/kg*m^2", "access": "ro", "registers": 12, "note": "Center of mass + moments of inertia 6D frame"},
    "module_type": {"code": "0x0500", "type": "enum", "unit": None, "access": "ro", "registers": 1, "note": None},
    "wp_lost_dst": {"code": "0x0528", "type": "float", "unit": "mm", "access": "rw", "registers": 2, "note": "Range 0.1-50, default 2"},
    "wp_release_delta": {"code": "0x0540", "type": "float", "unit": "mm", "access": "rw", "registers": 2, "note": "Range 1-50, default 2"},
    "grp_pos_margin": {"code": "0x0580", "type": "float", "unit": "mm", "access": "rw", "registers": 2, "note": "Workpiece position window tolerance; range 1-10, default 2"},
    "max_phys_stroke": {"code": "0x0588", "type": "float", "unit": "mm", "access": "ro", "registers": 2, "note": "Max physical stroke without fingers"},
    "grp_prepos_delta": {"code": "0x05A8", "type": "float", "unit": "mm", "access": "rw", "registers": 2, "note": "Range 1-50, default 5"},
    "min_pos": {"code": "0x0600", "type": "float", "unit": "mm", "access": "rw", "registers": 2, "note": None},
    "max_pos": {"code": "0x0608", "type": "float", "unit": "mm", "access": "rw", "registers": 2, "note": None},
    "zero_pos_ofs": {"code": "0x0610", "type": "float", "unit": "mm", "access": "rw", "registers": 2, "note": "Range -10000..10000, default 0"},
    "min_vel": {"code": "0x0628", "type": "float", "unit": "mm/s", "access": "ro", "registers": 2, "note": None},
    "max_vel": {"code": "0x0630", "type": "float", "unit": "mm/s", "access": "ro", "registers": 2, "note": None},
    "max_grp_vel": {"code": "0x0650", "type": "float", "unit": "mm/s", "access": "ro", "registers": 2, "note": None},
    "min_grp_force": {"code": "0x0658", "type": "float", "unit": "N", "access": "ro", "registers": 2, "note": None},
    "max_grp_force": {"code": "0x0660", "type": "float", "unit": "N", "access": "ro", "registers": 2, "note": "100% reference for grip force percentage"},
    "max_allow_force": {"code": "0x06A8", "type": "float", "unit": "N", "access": "ro", "registers": 2, "note": "EGU/EZU only: StrongGrip force ceiling"},
    "min_err_mot_volt": {"code": "0x0800", "type": "float", "unit": "V", "access": "ro", "registers": 2, "note": None},
    "max_err_mot_volt": {"code": "0x0808", "type": "float", "unit": "V", "access": "ro", "registers": 2, "note": None},
    "min_err_lgc_volt": {"code": "0x0810", "type": "float", "unit": "V", "access": "ro", "registers": 2, "note": None},
    "max_err_lgc_volt": {"code": "0x0818", "type": "float", "unit": "V", "access": "ro", "registers": 2, "note": None},
    "min_err_lgc_temp": {"code": "0x0820", "type": "float", "unit": "degC", "access": "ro", "registers": 2, "note": None},
    "max_err_lgc_temp": {"code": "0x0828", "type": "float", "unit": "degC", "access": "ro", "registers": 2, "note": None},
    "meas_lgc_temp": {"code": "0x0840", "type": "float", "unit": "degC", "access": "ro", "registers": 2, "note": None},
    "meas_lgc_volt": {"code": "0x0870", "type": "float", "unit": "V", "access": "ro", "registers": 2, "note": None},
    "meas_mot_volt": {"code": "0x0878", "type": "float", "unit": "V", "access": "ro", "registers": 2, "note": None},
    "min_wrn_mot_volt": {"code": "0x0880", "type": "float", "unit": "V", "access": "rw", "registers": 2, "note": "Range 19-36, default 21.6"},
    "max_wrn_mot_volt": {"code": "0x0888", "type": "float", "unit": "V", "access": "rw", "registers": 2, "note": "Range 19-36, default 26.4"},
    "min_wrn_lgc_volt": {"code": "0x0890", "type": "float", "unit": "V", "access": "rw", "registers": 2, "note": "Range 11-48, default 21.6"},
    "max_wrn_lgc_volt": {"code": "0x0898", "type": "float", "unit": "V", "access": "rw", "registers": 2, "note": "Range 11-48, default 26.4"},
    "min_wrn_lgc_temp": {"code": "0x08A0", "type": "float", "unit": "degC", "access": "ro", "registers": 2, "note": None},
    "max_wrn_lgc_temp": {"code": "0x08A8", "type": "float", "unit": "degC", "access": "ro", "registers": 2, "note": None},
    "serial_no_txt": {"code": "0x1000", "type": "char[16]", "unit": None, "access": "ro", "registers": 8, "note": None},
    "order_no_txt": {"code": "0x1008", "type": "char[16]", "unit": None, "access": "ro", "registers": 8, "note": None},
    "serial_no_num": {"code": "0x1020", "type": "uint32", "unit": None, "access": "ro", "registers": 2, "note": None},
    "fw_app_date": {"code": "0x1100", "type": "char[12]", "unit": None, "access": "ro", "registers": 6, "note": None},
    "fw_app_time": {"code": "0x1108", "type": "char[9]", "unit": None, "access": "ro", "registers": 5, "note": None},
    "fw_app_ver_num": {"code": "0x1110", "type": "uint16", "unit": None, "access": "ro", "registers": 1, "note": None},
    "fw_app_ver_txt": {"code": "0x1118", "type": "char[22]", "unit": None, "access": "ro", "registers": 11, "note": None},
    "mac_addr": {"code": "0x1138", "type": "uint8[3]", "unit": None, "access": "ro", "registers": 3, "note": None},
    "baudrate": {"code": "0x11A0", "type": "uint32", "unit": "bit/s", "access": "rw", "registers": 2, "note": "19200/115200/230400/460800/921600, default 115200; needs restart to apply"},
    "slave_id": {"code": "0x11A8", "type": "uint8", "unit": None, "access": "rw", "registers": 1, "note": "Range 1-247, default 12"},
    "enable_softreset": {"code": "0x1330", "type": "bool", "unit": None, "access": "rw", "registers": 1, "note": None},
    "system_uptime": {"code": "0x1400", "type": "uint32", "unit": "s", "access": "ro", "registers": 2, "note": "Seconds since last (re)start"},
}

# name -> hex code, derived from PARAMETER_CATALOG for validation lookups.
PARAMETER_ADDRESSES = {name: info["code"] for name, info in PARAMETER_CATALOG.items()}


WARNING_CODES = {
    0x90: "WRN_LGC_TEMP_LO: Logic temperature too low",
    0x92: "WRN_MOT_TEMP_LO: Motor temperature too low",
    0x93: "WRN_MOT_TEMP_HI: Motor temperature too high",
    0x94: "WRN_NOT_FEASIBLE: Command not feasible (see additional_code)",
    0x95: "WRN_POS_LIMIT: Jog ended at min/max position",
    0x96: "WRN_LGC_VOLT_LO: Logic supply voltage too low",
    0x97: "WRN_LGC_VOLT_HI: Logic supply voltage too high",
    0x98: "WRN_MOT_VOLT_LO: Motor supply voltage too low",
    0x99: "WRN_MOT_VOLT_HI: Motor supply voltage too high",
}

ERROR_CODES = {
    0x28: "ERR_BT_FAILED: Brake test failed",
    0x6C: "ERR_MOT_TEMP_LO: Motor temperature too low",
    0x6D: "ERR_MOT_TEMP_HI: Motor temperature too high",
    0x70: "ERR_LGC_TEMP_LO: Logic temperature too low",
    0x71: "ERR_LGC_TEMP_HI: Logic temperature too high",
    0x72: "ERR_LGC_VOLT_LO: Logic supply voltage too low",
    0x73: "ERR_LGC_VOLT_HI: Logic supply voltage too high",
    0x74: "ERR_MOT_VOLT_LO: Motor supply voltage too low",
    0x75: "ERR_MOT_VOLT_HI: Motor supply voltage too high",
    0xD5: "ERR_SOFT_LOW: Lower software limit reached/exceeded",
    0xD6: "ERR_SOFT_HIGH: Upper software limit reached/exceeded",
    0xD9: "ERR_FAST_STOP: Fast stop triggered (also post-boot state)",
    0xE4: "ERR_TOO_FAST: Max velocity exceeded by factor 1.2",
    0xEF: "ERR_COMM_LOST: Communication with controller lost",
    0xF1: "ERR_MOV_ABORT_TO: Positioning timed out",
    0xF4: "ERR_MOVE_BLOCKED: Drive blocked",
}

def _parse_diagnosis_code(code: str) -> int | None:
    """Parse a GripperState hex code string (e.g. "0x94" or "warning_code: 0x94") into an int."""
    if not code:
        return None
    token = code.rsplit(":", 1)[-1].strip()
    try:
        return int(token, 16)
    except ValueError:
        return None


def _decode_float_payload(payload: str) -> float | None:
    """Best-effort decode of a 4-byte hyphenated hex payload (e.g. "00-00-C8-41") as a little-endian float32.

    Modbus RTU parameter values are little-endian per the SCHUNK commissioning
    manual (only Modbus protocol data such as register addresses/counts is
    big-endian), e.g. "00-00-C8-41" -> 25.0 for max_grp_vel.
    """
    try:
        data = bytes(int(byte, 16) for byte in payload.split("-"))
    except ValueError:
        return None
    if len(data) != 4:
        return None
    try:
        return struct.unpack("<f", data)[0]
    except struct.error:
        return None


NOT_FEASIBLE_CODES = {
    0x00: "No warning present",
    0x01: "Unsupported acyclic IO-Link function requested",
    0x02: "App/Factory/Back-To-Box reset only allowed in error state",
    0x03: "Shutdown not allowed from current state",
    0x04: "Restart not allowed from current state",
    0x05: "Factory reset not allowed from current state",
    0x06: "Function triggered while in error state",
    0x08: "Restart disabled via enable_softreset parameter",
    0x09: "fast stop (bit 0) was reset when command issued",
    0x0A: "Several commands requested at once",
    0x0C: "Illegal function during brake test",
    0x0D: "Manual removal only allowed in error state",
    0x0E: "Release issued while no workpiece held",
    0x0F: "Illegal function while holding a workpiece",
    0x10: "Target position outside limits",
    0x11: "Invalid workpiece-position/grip-direction combo",
    0x12: "Illegal function during gripping",
    0x13: "Velocity outside limits",
    0x15: "Illegal function during positioning",
    0x1D: "Illegal function during manual-removal state",
    0x1E: "Illegal function while releasing workpiece",
    0x22: "min_pos write out of range",
    0x23: "min_pos > max_pos",
    0x24: "max_pos write out of range",
    0x25: "max_pos < min_pos",
    0x26: "Release target outside limits",
    0x27: "Release target lies inside the workpiece",
    0x28: "GPE requested on a module without GPE",
    0x29: "Grip force outside allowed range",
}


class RosBridge:
    """Bridge between the MCP server and the ROS 2 gripper driver node."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()

        self._node = Node("gripper_mcp_bridge")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        # Spin in background thread
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        # Lifecycle service clients
        self._get_state_client = self._node.create_client(
            GetState, f"{DRIVER_NODE}/get_state"
        )
        self._change_state_client = self._node.create_client(
            ChangeState, f"{DRIVER_NODE}/change_state"
        )

        # Cached topic data
        self._gripper_states: dict[str, GripperState] = {}
        self._joint_states: dict[str, JointState] = {}
        self._connection_state: ConnectionState | None = None

        # Topic subscribers (created lazily)
        self._subscriptions_created = False

    def _spin(self):
        while rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def _call_service(self, client, request, timeout: float = SERVICE_TIMEOUT):
        """Call a ROS 2 service synchronously."""
        try:
            logger.debug(f"Waiting for service {client.srv_name} to be available...")
            if not client.wait_for_service(timeout_sec=5.0):
                logger.error(f"Service {client.srv_name} not available after 5s timeout")
                return None
            
            logger.debug(f"Service {client.srv_name} available, calling with request: {request}")
            future = client.call_async(request)
            start = time.time()
            while not future.done():
                if time.time() - start > timeout:
                    logger.error(f"Service {client.srv_name} call timed out after {timeout}s")
                    return None
                time.sleep(0.01)
            
            result = future.result()
            logger.debug(f"Service {client.srv_name} returned: {result}")
            return result
        except Exception as e:
            logger.exception(f"Exception calling service {client.srv_name}: {e}")
            return None

    def _get_service_type(self, service_name: str) -> str | None:
        """Return the advertised ROS type for a service, if it is available."""
        for name, types in self._node.get_service_names_and_types():
            if name == service_name:
                return types[0] if types else None
        return None

    def _has_gpe(self, gripper_id: str) -> bool:
        """Determine if gripper has GPE capability based on gripper ID.
        
        Gripper ID format: TYPE_SIZE_MB_M/N_B_ID
        M = GPE-equipped (Grip force & position maintenance)
        N = No GPE
        """
        # Check for 'M' in the gripper ID (indicates GPE capability)
        return '_M_' in gripper_id

    def _ensure_subscriptions(self):
        """Create topic subscriptions for gripper state monitoring."""
        if self._subscriptions_created:
            return
        self._node.create_subscription(
            ConnectionState,
            f"{DRIVER_NODE}/connection_state",
            self._connection_state_cb,
            1,
        )
        self._subscriptions_created = True

    def _connection_state_cb(self, msg: ConnectionState):
        self._connection_state = msg
        # Dynamically subscribe to per-gripper topics
        for gripper_id in msg.grippers:
            if gripper_id not in self._gripper_states:
                self._gripper_states[gripper_id] = None
                self._node.create_subscription(
                    GripperState,
                    f"{DRIVER_NODE}/{gripper_id}/gripper_state",
                    lambda m, gid=gripper_id: self._gripper_state_cb(gid, m),
                    1,
                )
            if gripper_id not in self._joint_states:
                self._joint_states[gripper_id] = None
                self._node.create_subscription(
                    JointState,
                    f"{DRIVER_NODE}/{gripper_id}/joint_states",
                    lambda m, gid=gripper_id: self._joint_state_cb(gid, m),
                    1,
                )

    def _gripper_state_cb(self, gripper_id: str, msg: GripperState):
        self._gripper_states[gripper_id] = msg

    def _joint_state_cb(self, gripper_id: str, msg: JointState):
        self._joint_states[gripper_id] = msg

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def get_state(self) -> str:
        self._ensure_subscriptions()
        req = GetState.Request()
        resp = self._call_service(self._get_state_client, req)
        if resp is None:
            return json.dumps({"error": "Driver node not reachable. Is the driver running?"})
        state_name = STATE_NAMES.get(resp.current_state.id, "unknown")
        return json.dumps({"state": state_name, "state_id": resp.current_state.id})

    def change_state(self, transition: str) -> str:
        """Apply a lifecycle transition, deactivating before cleanup if needed."""
        if transition not in TRANSITIONS:
            return json.dumps({"error": f"Unknown transition: {transition}"})
        if transition == "cleanup":
            state_response = self._call_service(self._get_state_client, GetState.Request())
            if state_response is None:
                return json.dumps({"error": "Driver node not reachable. Is the driver running?"})
            current_state = state_response.current_state.id
            if current_state == 0:
                return json.dumps({"error": "Driver state is unknown. Cleanup not sent."})
            if current_state == 3:
                deactivate_response = self._call_service(
                    self._change_state_client,
                    self._lifecycle_request("deactivate"),
                )
                if deactivate_response is None or not deactivate_response.success:
                    return json.dumps({
                        "success": False,
                        "transition": "deactivate",
                        "message": "Unable to deactivate driver. Cleanup not sent.",
                    })
        req = ChangeState.Request()
        req.transition.id = TRANSITIONS[transition]
        resp = self._call_service(self._change_state_client, req)
        if resp is None:
            return json.dumps({"error": "Driver node not reachable. Is the driver running?"})
        if resp.success:
            return json.dumps({"success": True, "transition": transition})
        return json.dumps({"success": False, "transition": transition})

    @staticmethod
    def _lifecycle_request(transition: str) -> ChangeState.Request:
        request = ChangeState.Request()
        request.transition.id = TRANSITIONS[transition]
        return request

    # ------------------------------------------------------------------
    # Setup & discovery (unconfigured state)
    # ------------------------------------------------------------------

    def scan_grippers(self, scan_modbus: bool, serial_port: str) -> str:
        client = self._node.create_client(ScanGrippers, f"{DRIVER_NODE}/scan")
        req = ScanGrippers.Request()
        req.scan_modbus = scan_modbus
        req.serial_port = serial_port
        resp = self._call_service(client, req, timeout=30.0)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": "Scan service not available. Is the driver in UNCONFIGURED state?"})
        grippers = []
        for name, conn in zip(resp.grippers, resp.connections):
            entry = {"type": name}
            if conn.host:
                entry["host"] = conn.host
                entry["port"] = conn.port
            if conn.serial_port:
                entry["serial_port"] = conn.serial_port
                entry["device_id"] = conn.device_id
            grippers.append(entry)
        return json.dumps({"grippers": grippers})

    def add_gripper(self, host: str, port: int, serial_port: str, device_id: int) -> str:
        client = self._node.create_client(AddGripper, f"{DRIVER_NODE}/add_gripper")
        req = AddGripper.Request()
        req.gripper.host = host
        req.gripper.port = port
        req.gripper.serial_port = serial_port
        req.gripper.device_id = device_id
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": "Add gripper service not available. Is the driver in UNCONFIGURED state?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def locate_gripper(self, host: str, port: int, serial_port: str, device_id: int) -> str:
        client = self._node.create_client(LocateGripper, f"{DRIVER_NODE}/locate_gripper")
        req = LocateGripper.Request()
        req.gripper.host = host
        req.gripper.port = port
        req.gripper.serial_port = serial_port
        req.gripper.device_id = device_id
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": "Locate gripper service not available. Is the driver in UNCONFIGURED state?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def show_configuration(self) -> str:
        client = self._node.create_client(ShowConfiguration, f"{DRIVER_NODE}/show_configuration")
        req = ShowConfiguration.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": "Show configuration service not available. Is the driver in UNCONFIGURED state?"})
        config = []
        for gripper in resp.configuration:
            entry = {}
            if gripper.host:
                entry["host"] = gripper.host
                entry["port"] = gripper.port
            if gripper.serial_port:
                entry["serial_port"] = gripper.serial_port
                entry["device_id"] = gripper.device_id
            config.append(entry)
        return json.dumps({"configuration": config})

    def reset_grippers(self) -> str:
        client = self._node.create_client(Trigger, f"{DRIVER_NODE}/reset_grippers")
        req = Trigger.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": "Reset service not available. Is the driver in UNCONFIGURED state?"})
        return json.dumps({"success": resp.success})

    def save_configuration(self) -> str:
        client = self._node.create_client(Trigger, f"{DRIVER_NODE}/save_configuration")
        req = Trigger.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": "Save configuration service not available. Is the driver in UNCONFIGURED state?"})
        return json.dumps({"success": resp.success})

    def load_previous_configuration(self) -> str:
        client = self._node.create_client(Trigger, f"{DRIVER_NODE}/load_previous_configuration")
        req = Trigger.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": "Load configuration service not available. Is the driver in UNCONFIGURED state?"})
        return json.dumps({"success": resp.success})

    # ------------------------------------------------------------------
    # Info (configured state and above)
    # ------------------------------------------------------------------

    def list_grippers(self) -> str:
        client = self._node.create_client(ListGrippers, f"{DRIVER_NODE}/list_grippers")
        req = ListGrippers.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": "List grippers service not available. Is the driver in INACTIVE or ACTIVE state?"})
        return json.dumps({"grippers": list(resp.grippers)})

    def _read_parameter_float(self, gripper_id: str, parameter_name: str) -> float | None:
        parameter = PARAMETER_ADDRESSES[parameter_name]
        client = self._node.create_client(
            ReadGripperParameterRaw,
            f"{DRIVER_NODE}/{gripper_id}/_read_parameter_raw",
        )
        req = ReadGripperParameterRaw.Request()
        req.parameter = parameter
        req.length = 0
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None or not resp.success:
            return None
        value = _decode_float_payload(resp.payload)
        return value if value is not None and math.isfinite(value) else None

    def _validate_motion(
        self,
        gripper_id: str,
        *,
        position: float | None = None,
        relative_position: bool = False,
        velocity: float | None = None,
        force: int | None = None,
        grip: bool = False,
    ) -> str | None:
        values: dict[str, float] = {}
        required = ["min_pos", "max_pos"] if position is not None else []
        if velocity is not None:
            required.append("max_grp_vel" if grip else "min_vel")
            if not grip:
                required.append("max_vel")
        for name in required:
            value = self._read_parameter_float(gripper_id, name)
            if value is None:
                return f"Unable to read gripper parameter '{name}' for validation. Command not sent."
            values[name] = value

        if position is not None:
            if not math.isfinite(position):
                return "Target position must be a finite number. Command not sent."
            target_mm = position * 1000.0
            if relative_position:
                actual_pos = self._read_parameter_float(gripper_id, "actual_pos")
                if actual_pos is None:
                    return "Unable to read actual position for validation. Command not sent."
                target_mm += actual_pos
            if not values["min_pos"] <= target_mm <= values["max_pos"]:
                return (
                    f"Target position {target_mm / 1000.0:g} m is outside the valid range "
                    f"[{values['min_pos'] / 1000.0:g}, {values['max_pos'] / 1000.0:g}] m. "
                    "Command not sent."
                )

        if velocity is not None:
            if not math.isfinite(velocity) or velocity < 0:
                return "Velocity must be a finite, non-negative number. Command not sent."
            velocity_mm_s = velocity * 1000.0
            if grip:
                min_velocity = 0.0
                max_velocity = values["max_grp_vel"]
            else:
                min_velocity = values["min_vel"]
                max_velocity = values["max_vel"]
            if not min_velocity <= velocity_mm_s <= max_velocity:
                return (
                    f"Velocity {velocity_mm_s:g} mm/s is outside the valid range "
                    f"[{min_velocity:g}, {max_velocity:g}] mm/s. Command not sent."
                )

        if force is not None:
            # StrongGrip (>100%) exists only on EGU/EZU and only on GPE-equipped
            # (M) modules, where GPE activates automatically for the grip. EGK
            # never has StrongGrip; it only offers BasicGrip/SoftGrip (<=100%).
            max_force = 100
            if gripper_id.startswith(("EGU_", "EZU_")) and self._has_gpe(gripper_id):
                max_force = 150 if gripper_id.startswith("EGU_70_") else 200
            if not 50 <= force <= max_force:
                return (
                    f"Grip force {force}% is outside the valid range [50, {max_force}]%. "
                    "Command not sent."
                )
        return None

    # ------------------------------------------------------------------
    # Gripper control (active state)
    # ------------------------------------------------------------------

    def move_to_absolute_position(
        self, gripper_id: str, position: float, velocity: float, use_gpe: bool
    ) -> str:
        validation_error = self._validate_motion(
            gripper_id, position=position, velocity=velocity
        )
        if validation_error:
            return json.dumps({"success": False, "message": validation_error})
        
        # Determine service type based on gripper's actual GPE capability, not the parameter
        gripper_has_gpe = self._has_gpe(gripper_id)
        
        if gripper_has_gpe:
            client = self._node.create_client(
                MoveToAbsolutePositionGPE,
                f"{DRIVER_NODE}/{gripper_id}/move_to_absolute_position",
            )
            req = MoveToAbsolutePositionGPE.Request()
            req.use_gpe = use_gpe
        else:
            client = self._node.create_client(
                MoveToAbsolutePosition,
                f"{DRIVER_NODE}/{gripper_id}/move_to_absolute_position",
            )
            req = MoveToAbsolutePosition.Request()
        req.position = position
        req.velocity = velocity
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def move_to_relative_position(
        self, gripper_id: str, position: float, velocity: float, use_gpe: bool
    ) -> str:
        validation_error = self._validate_motion(
            gripper_id, position=position, relative_position=True, velocity=velocity
        )
        if validation_error:
            return json.dumps({"success": False, "message": validation_error})
        
        # Determine service type based on gripper's actual GPE capability, not the parameter
        gripper_has_gpe = self._has_gpe(gripper_id)
        
        if gripper_has_gpe:
            client = self._node.create_client(
                MoveToRelativePositionGPE,
                f"{DRIVER_NODE}/{gripper_id}/move_to_relative_position",
            )
            req = MoveToRelativePositionGPE.Request()
            req.use_gpe = use_gpe
        else:
            client = self._node.create_client(
                MoveToRelativePosition,
                f"{DRIVER_NODE}/{gripper_id}/move_to_relative_position",
            )
            req = MoveToRelativePosition.Request()
        req.position = position
        req.velocity = velocity
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def grip(
        self,
        gripper_id: str,
        force: int,
        outward: bool,
        position: float | None,
        velocity: float | None,
        use_gpe: bool,
    ) -> str:
        validation_error = self._validate_motion(
            gripper_id,
            position=position,
            velocity=velocity,
            force=force,
            grip=True,
        )
        if validation_error:
            return json.dumps({"success": False, "message": validation_error})
        # Select the appropriate service type based on parameters
        srv_name = f"{DRIVER_NODE}/{gripper_id}/grip"
        if position is not None:
            srv_name = f"{DRIVER_NODE}/{gripper_id}/grip_at_position"

        service_type = self._get_service_type(srv_name)
        if service_type is None:
            return json.dumps({"error": f"Grip service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        service_type_name = service_type.rsplit("/", 1)[-1]

        request_types = {
            "Grip": (Grip, False, False),
            "GripWithGPE": (GripWithGPE, True, False),
            "GripWithVelocity": (GripWithVelocity, False, True),
            "GripWithVelocityAndGPE": (GripWithVelocityAndGPE, True, True),
            "GripAtPosition": (GripAtPosition, False, False),
            "GripAtPositionWithGPE": (GripAtPositionWithGPE, True, False),
            "GripAtPositionWithVelocity": (GripAtPositionWithVelocity, False, True),
            "GripAtPositionWithVelocityAndGPE": (GripAtPositionWithVelocityAndGPE, True, True),
        }
        request_info = request_types.get(service_type_name)
        if request_info is None:
            return json.dumps({"error": f"Unsupported grip service type '{service_type}' for gripper '{gripper_id}'."})

        request_class, service_has_gpe, service_has_velocity = request_info
        if velocity is not None and not service_has_velocity:
            return json.dumps({
                "success": False,
                "message": f"Gripper '{gripper_id}' exposes {service_type_name}, which does not support a gripping velocity.",
            })
        client = self._node.create_client(request_class, srv_name)
        req = request_class.Request()
        if service_has_gpe:
            req.use_gpe = use_gpe
        if position is not None:
            req.position = position
        if velocity is not None:
            req.velocity = velocity

        req.force = force
        req.outward = outward
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Grip service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        result = {
            "success": resp.success,
            "message": resp.message,
            "workpiece_gripped": resp.workpiece_gripped,
            "no_workpiece_detected": resp.no_workpiece_detected,
            "wrong_workpiece_gripped": resp.wrong_workpiece_gripped,
            "workpiece_lost": resp.workpiece_lost,
        }
        return json.dumps(result)

    def release(self, gripper_id: str, use_gpe: bool) -> str:
        srv_name = f"{DRIVER_NODE}/{gripper_id}/release"
        # Determine service type based on gripper's actual GPE capability, not the parameter
        gripper_has_gpe = self._has_gpe(gripper_id)
        
        if gripper_has_gpe:
            client = self._node.create_client(ReleaseWithGPE, srv_name)
            req = ReleaseWithGPE.Request()
            req.use_gpe = use_gpe
        else:
            client = self._node.create_client(Release, srv_name)
            req = Release.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Release service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def acknowledge(self, gripper_id: str) -> str:
        client = self._node.create_client(Trigger, f"{DRIVER_NODE}/{gripper_id}/acknowledge")
        req = Trigger.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Acknowledge service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def stop(self, gripper_id: str, use_gpe: bool) -> str:
        srv_name = f"{DRIVER_NODE}/{gripper_id}/stop"
        # Determine service type based on gripper's actual GPE capability, not the parameter
        gripper_has_gpe = self._has_gpe(gripper_id)
        
        if gripper_has_gpe:
            client = self._node.create_client(StopWithGPE, srv_name)
            req = StopWithGPE.Request()
            req.use_gpe = use_gpe
        else:
            client = self._node.create_client(Stop, srv_name)
            req = Stop.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Stop service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def fast_stop(self, gripper_id: str) -> str:
        client = self._node.create_client(Trigger, f"{DRIVER_NODE}/{gripper_id}/fast_stop")
        req = Trigger.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Fast stop service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def start_jogging(self, gripper_id: str, velocity: float, use_gpe: bool) -> str:
        validation_error = self._validate_motion(gripper_id, velocity=abs(velocity))
        if validation_error:
            return json.dumps({"success": False, "message": validation_error})
        srv_name = f"{DRIVER_NODE}/{gripper_id}/start_jogging"
        # Determine service type based on gripper's actual GPE capability, not the parameter
        gripper_has_gpe = self._has_gpe(gripper_id)
        
        if gripper_has_gpe:
            client = self._node.create_client(StartJoggingGPE, srv_name)
            req = StartJoggingGPE.Request()
            req.use_gpe = use_gpe
        else:
            client = self._node.create_client(StartJogging, srv_name)
            req = StartJogging.Request()
        req.velocity = velocity
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Jogging service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def stop_jogging(self, gripper_id: str) -> str:
        client = self._node.create_client(Trigger, f"{DRIVER_NODE}/{gripper_id}/stop_jogging")
        req = Trigger.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Stop jogging service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def prepare_for_shutdown(self, gripper_id: str) -> str:
        client = self._node.create_client(Trigger, f"{DRIVER_NODE}/{gripper_id}/prepare_for_shutdown")
        req = Trigger.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Prepare for shutdown service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def soft_reset(self, gripper_id: str) -> str:
        client = self._node.create_client(Trigger, f"{DRIVER_NODE}/{gripper_id}/soft_reset")
        req = Trigger.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Soft reset service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    def brake_test(self, gripper_id: str) -> str:
        client = self._node.create_client(Trigger, f"{DRIVER_NODE}/{gripper_id}/brake_test")
        req = Trigger.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Brake test service not available for gripper '{gripper_id}'. Is the driver ACTIVE and does this gripper support brake_test?"})
        return json.dumps({"success": resp.success, "message": resp.message})

    # ------------------------------------------------------------------
    # Status & diagnostics (active state)
    # ------------------------------------------------------------------

    def get_gripper_status(self, gripper_id: str) -> str:
        self._ensure_subscriptions()
        msg = self._gripper_states.get(gripper_id)
        if msg is None:
            return json.dumps({"error": f"No status available for gripper '{gripper_id}'. Is the driver ACTIVE and the gripper connected?"})

        # Resolve human-readable diagnosis descriptions
        error_code = _parse_diagnosis_code(msg.error_code)
        warning_code = _parse_diagnosis_code(msg.warning_code)
        additional_code = _parse_diagnosis_code(msg.additional_code)

        error_description = ERROR_CODES.get(error_code, f"Unknown error code {msg.error_code}" if error_code else None)
        warning_description = WARNING_CODES.get(warning_code, f"Unknown warning code {msg.warning_code}" if warning_code else None)
        additional_description = None
        if warning_code == 0x94:
            additional_description = NOT_FEASIBLE_CODES.get(additional_code, f"Unknown additional code {msg.additional_code}")

        # Summarize active conditions worth attention
        active_conditions = []
        if msg.bit7_error:
            active_conditions.append(f"ERROR: {error_description or 'unknown'}")
        if msg.bit6_warning:
            active_conditions.append(f"WARNING: {warning_description or 'unknown'}")
            if additional_description:
                active_conditions.append(f"NOT_FEASIBLE reason: {additional_description}")
        if msg.bit16_workpiece_lost:
            active_conditions.append("Workpiece lost during hold")
        if msg.bit17_wrong_workpiece_gripped:
            active_conditions.append("Wrong workpiece gripped (size outside expected window)")
        if msg.bit11_no_workpiece_detected:
            active_conditions.append("No workpiece detected")
        if msg.bit12_workpiece_gripped:
            active_conditions.append("Workpiece gripped")
        if msg.bit13_position_reached:
            active_conditions.append("Position reached")
        if msg.bit9_software_limit_reached:
            active_conditions.append("Software limit reached")
        if msg.bit0_ready_for_operation:
            active_conditions.append("Ready for operation")

        return json.dumps({
            "gripper_id": gripper_id,
            "error_code": msg.error_code,
            "error_description": error_description,
            "warning_code": msg.warning_code,
            "warning_description": warning_description,
            "additional_code": msg.additional_code,
            "additional_code_description": additional_description,
            "active_conditions": active_conditions,
            "ready_for_operation": msg.bit0_ready_for_operation,
            "control_authority_fieldbus": msg.bit1_control_authority_fieldbus,
            "ready_for_shutdown": msg.bit2_ready_for_shutdown,
            "not_feasible": msg.bit3_not_feasible,
            "command_successfully_processed": msg.bit4_command_successfully_processed,
            "command_received_toggle": msg.bit5_command_received_toggle,
            "warning": msg.bit6_warning,
            "error": msg.bit7_error,
            "released_for_manual_movement": msg.bit8_released_for_manual_movement,
            "software_limit_reached": msg.bit9_software_limit_reached,
            "no_workpiece_detected": msg.bit11_no_workpiece_detected,
            "workpiece_gripped": msg.bit12_workpiece_gripped,
            "position_reached": msg.bit13_position_reached,
            "workpiece_pre_grip_started": msg.bit14_workpiece_pre_grip_started,
            "workpiece_lost": msg.bit16_workpiece_lost,
            "wrong_workpiece_gripped": msg.bit17_wrong_workpiece_gripped,
            "grip_force_and_position_maintenance_activated": msg.bit31_grip_force_and_position_maintenance_activated,
        })

    def get_gripper_position(self, gripper_id: str) -> str:
        self._ensure_subscriptions()
        msg = self._joint_states.get(gripper_id)
        if msg is None:
            return json.dumps({"error": f"No position data for gripper '{gripper_id}'. Is the driver ACTIVE and the gripper connected?"})
        position = msg.position[0] if msg.position else None
        return json.dumps({"gripper_id": gripper_id, "position_m": position})

    def get_connection_state(self) -> str:
        self._ensure_subscriptions()
        # Wait briefly for first message if we haven't received one yet
        if self._connection_state is None:
            time.sleep(0.5)
        msg = self._connection_state
        if msg is None:
            return json.dumps({"error": "No connection state received. Is the driver running?"})
        connections = {}
        for gripper_id, connected in zip(msg.grippers, msg.connected):
            connections[gripper_id] = connected
        return json.dumps({"connections": connections})

    def get_gripper_specification(self, gripper_id: str) -> str:
        client = self._node.create_client(
            ShowGripperSpecification,
            f"{DRIVER_NODE}/{gripper_id}/show_specification",
        )
        req = ShowGripperSpecification.Request()
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Specification service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        if not resp.success:
            return json.dumps({"success": False, "message": resp.message})
        return json.dumps({
            "success": True,
            "gripper_id": gripper_id,
            "max_stroke_mm": resp.specification.max_stroke,
            "max_speed_mm_s": resp.specification.max_speed,
            "max_force_n": resp.specification.max_force,
            "serial_number": resp.specification.serial_number,
            "firmware_version": resp.specification.firmware_version,
            "device_id": resp.specification.device_id,
            "ip_address": resp.specification.ip_address,
        })

    def list_parameters(self) -> str:
        """Return the known Modbus parameter catalog (name, hex code, type, unit, access, registers)."""
        return json.dumps({"parameters": PARAMETER_CATALOG})

    def read_parameter_raw(self, gripper_id: str, parameter: str, length: int = 0) -> str:
        code = PARAMETER_ADDRESSES.get(parameter, parameter)
        if length == 0:
            length = PARAMETER_CATALOG.get(parameter, {}).get("registers", 0)
        client = self._node.create_client(
            ReadGripperParameterRaw,
            f"{DRIVER_NODE}/{gripper_id}/_read_parameter_raw",
        )
        req = ReadGripperParameterRaw.Request()
        req.parameter = code
        req.length = length
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Read parameter raw service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        if not resp.success:
            return json.dumps({"success": False, "gripper_id": gripper_id, "parameter": code, "message": "Failed to read parameter. Check the parameter address and length."})
        result = {"success": True, "gripper_id": gripper_id, "parameter": code, "payload": resp.payload}
        # Most numeric module parameters (e.g. velocities, positions, forces) are 4-byte floats.
        decoded_float = _decode_float_payload(resp.payload)
        if decoded_float is not None:
            result["decoded_float"] = decoded_float
        return json.dumps(result)

    def write_parameter_raw(self, gripper_id: str, parameter: str, payload: str, length: int = 0) -> str:
        code = PARAMETER_ADDRESSES.get(parameter, parameter)
        if length == 0:
            length = PARAMETER_CATALOG.get(parameter, {}).get("registers", 0)
        client = self._node.create_client(
            WriteGripperParameterRaw,
            f"{DRIVER_NODE}/{gripper_id}/_write_parameter_raw",
        )
        req = WriteGripperParameterRaw.Request()
        req.parameter = code
        req.length = length
        req.payload = payload
        resp = self._call_service(client, req)
        self._node.destroy_client(client)
        if resp is None:
            return json.dumps({"error": f"Write parameter raw service not available for gripper '{gripper_id}'. Is the driver ACTIVE?"})
        return json.dumps({"success": resp.success, "gripper_id": gripper_id, "parameter": code})
