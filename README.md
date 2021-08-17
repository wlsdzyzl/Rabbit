## Rabbit
> Run, rabbit, run. Dig the hole, forget the sun.

![](./demo.gif)

A small project to record the learning process of Lidar Slam. 

Rabbit is also used to compare several odometry and registration algorithms.

Download lego-loam datasets from https://drive.google.com/drive/folders/16p5UPUCZ1uK0U4XE-hJKjazTsRghEMJa. I transfer the `.bag` data into `.pcd` files. You can also download the pcd dataset from , which is used for the demo(`RunPCD.cpp`).

### Dependency
ROS, PCL 1.8.0

### Method

Rabbit supports LOAM, ICP, GICP(omp), NDT(omp) for lidar point cloud odometry and mapping. In LOAM method of rabbit, we have edge features, planar features and ground features. And the idea of ground feature is adopted from Lego-loam. For mapping strategy, we construct a large and dense volume using three ways: submap, sliding window and moving box (similar to LOAM).  Rabbit also supports naive distance based loop closure detection and scan context loop closure detection. A graph-optimization based on Ceres is used to maintain global consistency. In current version, we optimize the keyframe constraints and ground normal constraints.

### Result
**Suitable algorithm for odometry:** Loam (feature-based, faster), GICP, ICP
**Suitable algorithm for mapping:** NDT, ICP 
**Recommended combination:** Loam odometry + ndtomp mapping + naive loop closure detection + ground normal optimization,
```
rosrun rabbit RabbitRunPCD /media/wlsdzyzl/4986a128-c51f-4384-87a6-abf677343495/lidar-dataset/lego-loam/pcds loam ndtomp 2 1 1 0 1
```

### Future direction
- IMU factor
- Teaser++ and Ransac for global registration