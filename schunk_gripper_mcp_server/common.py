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

"""Transport-independent gripper knowledge shared by the ROS 2 and Ethernet bridges."""

import json
import math
import struct
from collections.abc import Callable

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

# Status double word bit -> name; matches the GripperState message fields "bit<N>_<name>".
STATUS_BITS = {
    0: "ready_for_operation",
    1: "control_authority_fieldbus",
    2: "ready_for_shutdown",
    3: "not_feasible",
    4: "command_successfully_processed",
    5: "command_received_toggle",
    6: "warning",
    7: "error",
    8: "released_for_manual_movement",
    9: "software_limit_reached",
    11: "no_workpiece_detected",
    12: "workpiece_gripped",
    13: "position_reached",
    14: "workpiece_pre_grip_started",
    16: "workpiece_lost",
    17: "wrong_workpiece_gripped",
    31: "grip_force_and_position_maintenance_activated",
}


def parse_diagnosis_code(code: str) -> int | None:
    """Parse a GripperState hex code string (e.g. "0x94" or "warning_code: 0x94") into an int."""
    if not code:
        return None
    token = code.rsplit(":", 1)[-1].strip()
    try:
        return int(token, 16)
    except ValueError:
        return None


def decode_float_payload(payload: str) -> float | None:
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


def has_gpe(gripper_id: str) -> bool:
    """Gripper IDs look like TYPE_SIZE_FIELDBUS_M/N_B_N; M = GPE-equipped, N = no GPE."""
    return "_M_" in gripper_id


def validate_motion(
    gripper_id: str,
    read_mm: Callable[[str], float | None],
    *,
    position: float | None = None,
    relative_position: bool = False,
    velocity: float | None = None,
    force: int | None = None,
    grip: bool = False,
) -> str | None:
    """Return an error message if the command is out of range, else None.

    `read_mm(name)` returns a catalog parameter in mm or mm/s, or None if unreadable.
    """
    values: dict[str, float] = {}
    required = ["min_pos", "max_pos"] if position is not None else []
    if velocity is not None:
        required.append("max_grp_vel" if grip else "min_vel")
        if not grip:
            required.append("max_vel")
    for name in required:
        value = read_mm(name)
        if value is None:
            return f"Unable to read gripper parameter '{name}' for validation. Command not sent."
        values[name] = value

    if position is not None:
        if not math.isfinite(position):
            return "Target position must be a finite number. Command not sent."
        target_mm = position * 1000.0
        if relative_position:
            actual_pos = read_mm("actual_pos")
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
        if gripper_id.startswith(("EGU_", "EZU_")) and has_gpe(gripper_id):
            max_force = 150 if gripper_id.startswith("EGU_70_") else 200
        if not 50 <= force <= max_force:
            return (
                f"Grip force {force}% is outside the valid range [50, {max_force}]%. "
                "Command not sent."
            )
    return None


def format_gripper_status(
    gripper_id: str,
    error_code: str,
    warning_code: str,
    additional_code: str,
    status: dict[str, bool],
) -> str:
    """Build the get_gripper_status JSON from diagnosis code strings and STATUS_BITS-named flags."""
    error = parse_diagnosis_code(error_code)
    warning = parse_diagnosis_code(warning_code)
    additional = parse_diagnosis_code(additional_code)

    error_description = ERROR_CODES.get(error, f"Unknown error code {error_code}" if error else None)
    warning_description = WARNING_CODES.get(warning, f"Unknown warning code {warning_code}" if warning else None)
    additional_description = None
    if warning == 0x94:
        additional_description = NOT_FEASIBLE_CODES.get(additional, f"Unknown additional code {additional_code}")

    # Summarize active conditions worth attention
    active_conditions = []
    if status["error"]:
        active_conditions.append(f"ERROR: {error_description or 'unknown'}")
    if status["warning"]:
        active_conditions.append(f"WARNING: {warning_description or 'unknown'}")
        if additional_description:
            active_conditions.append(f"NOT_FEASIBLE reason: {additional_description}")
    if status["workpiece_lost"]:
        active_conditions.append("Workpiece lost during hold")
    if status["wrong_workpiece_gripped"]:
        active_conditions.append("Wrong workpiece gripped (size outside expected window)")
    if status["no_workpiece_detected"]:
        active_conditions.append("No workpiece detected")
    if status["workpiece_gripped"]:
        active_conditions.append("Workpiece gripped")
    if status["position_reached"]:
        active_conditions.append("Position reached")
    if status["software_limit_reached"]:
        active_conditions.append("Software limit reached")
    if status["ready_for_operation"]:
        active_conditions.append("Ready for operation")

    return json.dumps({
        "gripper_id": gripper_id,
        "error_code": error_code,
        "error_description": error_description,
        "warning_code": warning_code,
        "warning_description": warning_description,
        "additional_code": additional_code,
        "additional_code_description": additional_description,
        "active_conditions": active_conditions,
        **{name: status[name] for name in STATUS_BITS.values()},
    })
