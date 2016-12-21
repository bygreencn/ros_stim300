# stim300_ros
A simple package to use stim300 in ROS.

## Topic
The fused orientation data are published on the `imu/data` topic by default but can be modified via the `topic_name` parameter.

## Visualization
The data from the IMU can be seen via the 3D visualization node from the `stim300_ros` package. But this package subscribe to the `/imu` topic so the `topic_name` parameter of the must be set to `/imu`.

Install it :

        $ rosdep update
        $ rosdep install stim300_ros
        OR
        $ rosdep update
        $ rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

        $ catkin_make_isolated --install --use-ninja


        $ rosrun stim300_ros stim300lib.py

## Parameters
- `device_id`:(Optional) used to change the default `/dev/ttyUSB0` harware device file node.
- `topic_name`: (Optional) used to change the default `/imu/data` topic to publish the IMU data.
- `frame_id`: (Optional) used to change the default `imu_link` frame.

## Launch
The node has to be launched with the `stim300_ros.launch`.

        $ roslaunch stim300_ros stim300_ros.launch
### Launch File Default value
- `device_id`:the default is `/dev/ttyUSB0`.
- `topic_name`: the default is `/imu`.
- `frame_id`: the default is `map`.