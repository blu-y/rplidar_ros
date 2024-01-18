
##### ERROR GLIBCXX_*.*.** not found

증상  
```
oscar@oscar-lab:~/ros2_ws$ ros2 launch rplidar_ros view_rplidar_s1_launch.py 
[INFO] [launch]: All log files can be found below /home/oscar/.ros/log/2024-01-18-17-57-11-057947-oscar-lab-21134
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rplidar_node-1]: process started with pid [21135]
[INFO] [rviz2-2]: process started with pid [21137]
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.26' not found (required by /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `CXXABI_1.3.11' not found (required by /opt/ros/humble/lib/librclcpp.so)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /opt/ros/humble/lib/librclcpp.so)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.26' not found (required by /opt/ros/humble/lib/librclcpp.so)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found (required by /opt/ros/humble/lib/librclcpp.so)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.22' not found (required by /opt/ros/humble/lib/librclcpp.so)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `CXXABI_1.3.13' not found (required by /opt/ros/humble/lib/librclcpp.so)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /opt/ros/humble/lib/librcpputils.so)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /lib/x86_64-linux-gnu/libspdlog.so.1)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.22' not found (required by /lib/x86_64-linux-gnu/libspdlog.so.1)
[rplidar_node-1] /home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node: /home/oscar/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /lib/x86_64-linux-gnu/libfmt.so.8)
[ERROR] [rplidar_node-1]: process has died [pid 21135, exit code 1, cmd '/home/oscar/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node --ros-args -r __node:=rplidar_node --params-file /tmp/launch_params_vnpr3x_0'].
```
원인  
local lib에는 라이브러리가 있어도 ros package 실행 시 anaconda lib에서 탐색하여 실행하는 것으로 보임.

해결법  
anaconda lib에 usr lib 링크
```
rm /home/xx/anaconda3/lib/libstdc++.so.6
cp /usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30 ~/anaconda3/lib
ln -s ~/anaconda3/lib/libstdc++.so.6.0.30 ~/anaconda3/lib/libstdc++.so.6
```


##### PCD Viewer[(Link)](https://manpages.ubuntu.com/manpages/focal/man1/pcl_pcd_viewer.1.html)
```
sudo apt install pcd-tools -y
pcl_viewer <PCD file>
```
