import json
import shlex
import subprocess
from turtle import ht
from turtle import ht
from typing import Any, Dict, List, Union

from ConverterHelper import ConverterHelper

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


def publish_ee_pose():

  topic_name = "/inv_solver_topic"
  data_type = "geometry_msgs/msg/Pose"

  ht1 = [
          [1, 0, 0, 500], 
          [0, 1, 0, 100], 
          [0, 0, 1, 1500],
          [0, 0, 0, 1]
        ]
  
  ht2 = [
        [1, 0, 0, -200], 
        [0, 1, 0, 300], 
        [0, 0, 1, 1200],
        [0, 0, 0, 1]
      ]

  pose_data = []

  pose_data.append({
    "position": {"x": ht1[0][3], "y": ht1[1][3], "z": ht1[2][3]},
    "orientation": ConverterHelper.rot_to_quat(ht1[0:3])
  })

  pose_data.append({
    "position": {"x": ht2[0][3], "y": ht2[1][3], "z": ht2[2][3]},
    "orientation": ConverterHelper.rot_to_quat(ht2[0:3])
  })

  # pose_data_1 = {
  #   "position": {"x": 500, "y": 100, "z": 1500},
  #   "orientation": {"x": 0.0, "y": 0.0, "z": 0, "w": 1}
  # }

  # pose_data_2 = {
  #   "position": {"x": -200, "y": 300, "z": 1200},
  #   "orientation": {"x": 0.0, "y": 0.0, "z": 0, "w": 1}
  # }

    # print(ros2_pub(topic_name, data_type, pose_data_1, once=True, dry_run=True))
    # ros2_pub(topic_name, data_type, pose_data_1, once=True)

  for pose in pose_data:

    print(ros2_pub(topic_name, data_type, pose, once=True, dry_run=True))
    ros2_pub(topic_name, data_type, pose, once=True)

def publish_joint_angles():

  topic_name = "/fw_solver_topic"
  data_type = "std_msgs/msg/Float32MultiArray"

  joints = []
  joints.append({"data": [11.30993247, 24.96464072, -1.56633144, 90.0, -66.60169073, -78.69006753]})
  joints.append({"data": [123.69006753, 51.44099437, -39.53090112, 90.0, -78.08990675, 33.69006753]})

  

  for joint_angle in joints:     
    print(ros2_pub(topic_name, data_type, joint_angle, once=True, dry_run=True))
    ros2_pub(topic_name, data_type, joint_angle, once=True)


# publish_ee_pose()
publish_joint_angles()

