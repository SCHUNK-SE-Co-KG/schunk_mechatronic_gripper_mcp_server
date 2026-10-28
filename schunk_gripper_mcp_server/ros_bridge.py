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
from schunk_gripper_mcp_server.common import (
    PARAMETER_ADDRESSES,
    PARAMETER_CATALOG,
    STATUS_BITS,
    decode_float_payload,
    format_gripper_status,
    has_gpe,
    validate_motion,
)


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


def is_driver_available(timeout_sec: float) -> bool:
    """Return True if the driver's lifecycle get_state service appears within timeout_sec."""
    if not rclpy.ok():
        rclpy.init()
    node = Node("gripper_mcp_driver_probe")
    try:
        client = node.create_client(GetState, f"{DRIVER_NODE}/get_state")
        return client.wait_for_service(timeout_sec=timeout_sec)
    finally:
        node.destroy_node()


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
        return has_gpe(gripper_id)

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
        value = decode_float_payload(resp.payload)
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
        return validate_motion(
            gripper_id,
            lambda name: self._read_parameter_float(gripper_id, name),
            position=position,
            relative_position=relative_position,
            velocity=velocity,
            force=force,
            grip=grip,
        )

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

        status = {name: getattr(msg, f"bit{bit}_{name}") for bit, name in STATUS_BITS.items()}
        return format_gripper_status(
            gripper_id, msg.error_code, msg.warning_code, msg.additional_code, status
        )

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
        decoded_float = decode_float_payload(resp.payload)
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
