# CamVox_MeiyuanXiang
CamVox相关论文、代码中文注释以及代码改动

# 参考
https://github.com/ISEE-Technology/CamVox  

# 环境
1. Ubuntu（测试了Ubuntu16.04.5、Ubuntu18.04）  
2. ROS (测试了kinetic、melodic)  
3. Opencv (测试了OpenCV 3.4.1)  
4. PCL（测试了pcl1.9）  
5. Pangolin（测试了Pangolin0.6）  
6. Ceres Solver（测试ceres-solver-1.14.0）  
7. Eigen（测试了Eigen3.3.4）  
8. livox_ros_driver  
9. MVS camera SDK and Ros driver    

# 编译
1. 下载源码 git clone https://github.com/MeiyuanXiang/CamVox_MeiyuanXiang.git  
2. 将CamVox_MeiyuanXiang\src下的CamVox拷贝到ros工程空间src文件夹内，例如~/catkin_ws/src/  
3. cd ~/catkin_ws/src  
4. cd CamVox/isee-camvox && chmod a+x build.sh && chmod a+x build_ros.sh  
5. ./build.sh  
6. ./build_ros.sh  
7. source ~/catkin_ws/devel/setup.bash  
8. cd CamVox/isee-camvox/Vocabulary  
9. tar zxvf ORBvoc.txt.tar.gz  

# 数据
https://drive.google.com/file/d/1q58MxqcAAKw2sOPwB3WFENcGBcBGSXbz/view?usp=sharing  

# 运行
1. SUSTech campus with loop closure  
cd ~/catkin_ws/src/CamVox/  
chmod +x run.sh  
./run.sh  
2. SUSTech campus with loop closure  
roscore  
cd CamVox/isee-camvox  
rosrun online camvox Vocabulary/ORBvoc.bin camvox/online/Livox.yaml  
rosbag play CamVox.bag  
