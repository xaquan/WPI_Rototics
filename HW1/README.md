## ROS 2 Package: `hw1_pubsub`

This repository contains a ROS 2 package named **`hw1_pubsub`** that demonstrates a simple **publisher/subscriber** system.

### Nodes

- **Publisher Node**
  - Node name: `hw1_publisher`
  - Executable: `talker`
  - Publishes to topic: `publish_integer`
  - Behavior: publishes an increasing integer **once every second**

- **Subscriber Node**
  - Node name: `hw1_subscriber`
  - Executable: `listener`
  - Subscribes to topic: `publish_integer`
  - Behavior: prints the received integer and whether it is **odd** or **even**

---

## How to Build and Run

### 1) Build the workspace
```bash
cd ./HW1

rosdep install -i --from-path src --rosdistro humble -y
colcon build

### 2) Build the workspace
```bash
source install/setup.bash
ros2 run hw1_pubsub talker
