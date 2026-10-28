# SCHUNK Mechatronic Gripper MCP Server
MCP Server to control SCHUNK EGU, EGK and EZU grippers

## Prerequisites

This MCP server requires:

- Python 3.12 or newer
- A supported SCHUNK EGU, EGK, or EZU gripper
- Optional: ROS 2 with the SCHUNK ROS 2 driver and its interface packages

The server talks to grippers in one of two modes, chosen automatically on the
first tool call (`get_connection_mode` reports which one is active):

- **ROS 2**: used if the SCHUNK ROS 2 driver is running at `/schunk/driver`.
- **Ethernet**: otherwise, the server talks to Ethernet grippers directly over
  HTTP/JSON using `schunk_gripper_library` (pinned to driver release `v3.1.0`).
  Modbus RTU grippers are not supported in this mode.

For ROS 2 mode, install and build the ROS 2 driver by following the instructions in the
[SCHUNK mechatronic gripper ROS 2 driver repository](https://github.com/SCHUNK-SE-Co-KG/schunk_mechatronic_gripper).

After installing the driver, make sure its ROS 2 workspace has been built and
that you know the path to its generated `install/setup.bash` file.

## Installation

Clone this repository, create a Python virtual environment, and install the
MCP server package:

```bash
git clone https://github.com/SCHUNK-SE-Co-KG/schunk_mechatronic_gripper_mcp_server.git
cd schunk_mechatronic_gripper_mcp_server

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install .
```

The package installs the dependencies declared in `pyproject.toml`, including
`schunk_gripper_library` directly from the driver's GitHub repository (`git`
and network access are required during installation).
ROS 2 Python packages such as `rclpy` are provided by ROS 2 and must be
installed through the ROS 2 setup described in the driver repository.

Install from a shell where the ROS 2 driver workspace is **not** sourced.
Otherwise pip finds the workspace's copy of `schunk_gripper_library` on
`PYTHONPATH` and skips installing it into the virtual environment, which
breaks Ethernet mode when the server later runs without ROS 2.

## Ethernet mode (without ROS 2)

Start the server without sourcing ROS 2:

```bash
.venv/bin/schunk-gripper-mcp-server
```

The same setup tools are used in both modes: scan or add grippers. With the ROS 2 driver, continue with
`configure_driver` and `activate_driver`. In Ethernet mode, grippers connect when added, so control
tools can be used right away. Nothing is scanned or
loaded automatically on startup. In Ethernet mode, `save_configuration`
defaults to `configuration.json` in the MCP server folder, separate from the
ROS driver's `/var/tmp/schunk_gripper/configuration.json`. You can pass a file
path with `save_configuration(location="/path/to/grippers.json")`; the MCP
server remembers that path for `load_previous_configuration` after restart.
Pass a location to `load_previous_configuration` to load a different file once.

## Setup and startup

In the terminal used to start the MCP server, source both the ROS 2 setup and
the setup file from the driver workspace. Replace the paths with the locations
used on your system:

```bash
source /opt/ros/<ros2-distribution>/setup.bash
source /path/to/schunk_mechatronic_gripper/install/setup.bash
source .venv/bin/activate
```

Start the MCP server with:

```bash
schunk-gripper-mcp-server
```

The ROS 2 driver node must be running and available at `/schunk/driver` before
using the MCP tools. The MCP server communicates with that node through ROS 2;
it does not start the driver automatically.

The first setup sequence is performed through the MCP tools:

1. Check the driver state with `get_driver_state`.
2. Discover grippers with `scan_grippers`, or add one directly with
	 `add_gripper`.
3. Configure the driver with `configure_driver`.
4. Activate the driver with `activate_driver`.

## Start in VS Code

This repository includes a VS Code MCP configuration at
`.vscode/mcp.json`. It starts the server through `bash`, first sourcing the
ROS 2 setup file and the setup file from the SCHUNK driver workspace.

Before starting the server, open `.vscode/mcp.json` and replace the example
paths with the paths on your system:

```json
{
	"servers": {
		"schunk-gripper": {
			"command": "bash",
			"args": [
				"-c",
				"source /opt/ros/jazzy/setup.bash && source /path/to/ros2_ws/install/setup.bash && /path/to/schunk_mechatronic_gripper_mcp_server/.venv/bin/schunk-gripper-mcp-server"
			]
		}
	}
}
```

The command must point to the Python environment where this MCP server was
installed. The ROS 2 distribution and driver workspace paths must also match
your installation. Keep the ROS 2 and driver setup commands in the same shell
command so the MCP server can import `rclpy` and
`schunk_gripper_interfaces`.

To start the server in VS Code:

1. Open this repository as a folder in VS Code.
2. Open the Chat view.
3. Open the Chat view's Configure Chat or tools menu and select
	 `schunk-gripper`.
4. Start the server when VS Code prompts you, or run `MCP: List Servers` from
	 the Command Palette and choose `Start` for `schunk-gripper`.

The server will then be available to the VS Code Chat agent through the MCP
tools. The ROS 2 driver node must already be running at `/schunk/driver`.

See the driver repository for gripper communication, network, serial-port, and
hardware setup requirements.

## License

This project is licensed under the GNU General Public License v3.0 or later.
See [LICENSE](LICENSE) for the complete license text.
