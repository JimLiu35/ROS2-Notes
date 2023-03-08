# construct an simple urdf file

To construct the robot in rviz and gazebo these simulation software, we need to construct urdf file to define the properties of robot’s links and joint. 

### Instruction and reference for urdf file

[urdf/XML - ROS Wiki](http://wiki.ros.org/urdf/XML)

### `rsp.urdf`

```cpp
<robot name = "RSP">

  <link name = "base_link">
    <visual>
      <origin xyz = "0 0 0.25"/>
      <geometry>
        <cylinder length = "0.5" radius = "0.1"/>
      </geometry>
      <material name = "blue">
        <color rgba = " 0.0 0.0 0.0 1.0"/>
    </material>
    </visual>
  </link>

  <link name = "upper_link">
    <visual>
      <origin xyz = "0 0 0.25"/>
      <geometry>
      <cylinder length = "0.5" radius = "0.1"/>
    </geometry>
    <material name = "white">
      <color rgba = " 0.0 0.0 0.0 1.0"/>
    </material>
    </visual>
  </link>

  <link name = "tool">
  <joint name = "shoulder" type = "revolute">
    <parent link = "base_link"/>
    <child link = "upper_link"/>
    <origin xyz = "0.0 0.0 0.5" rpy = "0 0 0"/>
    <limit lower = "-1" upper = "1" effort = "10" velocity = "2"/>
  </joint>

  </link>
</robot>
```

### pull out the joint state gui

```cpp
sudo apt-get install joint_state_publisher_gui 

// open the gui 
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### Publisher the node

```cpp
// publish the node robot_description 
ros2 run robot_state_publisher robot_state_publisher --ros-args --param robot_description:="`cat /home/huan/test_ws/src/lecture/rsp_week05/urdf/rsp.urdf`" 

```

### Open the rviz2

```cpp
//launch the rviz2
ros2 run rviz2 rviz2 
```

1. add **`robotmodel`** 
2. change **Description source → `Topic`**
3. change **description topic → `robot_description`**
4. change the bar of shoulder to see the joint rotate.

## two robot arm share same urdf file

---

### Left Robot

```cpp
// mapping namespace as left and frame prefix add left 
 ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/left --param robot_description:="`cat /home/huan/test_ws/src/lecture/rsp_week05/urdf/rsp.urdf`" --param frame_prefix:=left/

ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args __ns:=/left
```

### Right Robot

```cpp
// mapping namespace as left and frame prefix add left 
 ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/right --param robot_description:="`cat /home/huan/test_ws/src/lecture/rsp_week05/urdf/rsp.urdf`" --param frame_prefix:=right/

ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args __ns:=/right
```

### In Rviz2

1. add **`robotmodel` -> rename right and left** 
2. change **Description source → `Topic`**
3. change **description topic → `/left/robot_description`**
4. change prefix to `/left` and `/right`
5. change the bar of shoulder to see the joint rotate.

We need to build `world frame` then we can connect left and right arm.

```cpp
ros2 run tf2_ros static_transform_publisher 0.5 0 0 0 0 0 world /right/basse_link

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world /left/basse_link
```

Spin six of command in different terminals.
