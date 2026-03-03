import json
import shlex
import subprocess
from typing import Any, Dict, List, Union

def ros2_pub(topic_name: str,
             msg_type: str,
             data: Union[Dict[str, Any], List[Any], str],
             once: bool = True,
             rate_hz: float | None = None,
             dry_run: bool = False) -> str:
    """
    Publish a ROS2 message using the CLI.

    Args:
      topic_name: e.g. "/ee_pose"
      msg_type:   e.g. "geometry_msgs/msg/Pose"
      data:       dict/list (recommended) or a string payload
      once:       True -> publish once (-1)
      rate_hz:    if set, publishes at rate (-r). If once=True, rate is ignored.
      dry_run:    True -> don't execute, just return the command string.

    Returns:
      The shell-safe command string.
    """

    cmd = ["ros2", "topic", "pub"]

    if once:
        cmd += ["-1"]
    elif rate_hz is not None:
        cmd += ["-r", str(rate_hz)]

    # payload: use JSON (valid YAML), so it's safe for ros2 CLI
    if isinstance(data, (dict, list)):
        payload = json.dumps(data)
    else:
        payload = str(data)

    cmd += [topic_name, msg_type, payload]

    cmd_str = " ".join(shlex.quote(x) for x in cmd)

    if not dry_run:
        subprocess.run(cmd, check=True)

    return cmd_str

topic_name = "/inv_solver_topic"
data_type = "geometry_msgs/msg/Pose"
pose_data_1 = {
  "position": {"x": 500, "y": 100, "z": 1500},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0, "w": 1}
}

pose_data_2 = {
  "position": {"x": -200, "y": 300, "z": 1200},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0, "w": 1}
}

print(ros2_pub(topic_name, data_type, pose_data_1, once=True, dry_run=True))
ros2_pub(topic_name, data_type, pose_data_1, once=True)

print(ros2_pub(topic_name, data_type, pose_data_2, once=True, dry_run=True))
ros2_pub(topic_name, data_type, pose_data_2, once=True)

