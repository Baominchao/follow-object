### 启动目标检测或者人体姿态跟随功能
1. rosrun jetson-inference detection.py
2. roslaunch jetson-inference follower2_camera.launch
### 启动AR码跟随功能
1. 打印docs文件夹word文档
2. roslaunch jetson-inference astraproplus_ar_track_alvar.launch
3. roslaunch jetson-inference follower2_ar.launch