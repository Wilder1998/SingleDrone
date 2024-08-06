
# FY使用手册
## 1. 未知环境

```bash
# 功能：在未知环境下的雷达定位和建图，将保存一个可以拿来定位的地图new.pcd
roslaunch fast_lio mapping_mid360.launch
# 功能：给FY地面站发布实时扫到的点云
roslaunch fast_lio fy_series_publisher.launch
```



## 2. 已知环境
### 2.1. 将上一步建的```new.pcd```改名为```origin.pcd```

### 2.2. 跑代码

```bash
# 功能：基于已知环境的点云地图origin.pcd和当前雷达静止时获得的点云进行GICP匹配，获得雷达在已知地图上的初始位置
roslaunch fast_lio initial_align.launch
# 功能：在已知环境的点云地图origin.pcd下进行定位（和incremental建图）
roslaunch fast_lio odom_mid360_with_map.launch
# 如果需要向地面站传输点云，则可以加入这个
roslaunch fast_lio fy_series_publisher.launch
```
### 2.3. 设置初值
如果需要从环境变量中获取位置初值，和之前的使用方式类似，将环境变量里写的初值位置写到```initial_align.launch```里的对应位置即可：
```bash
	<!-- 环境变量！！！！！！！ -->
	<param name="wxx/initial_align/Init_Body_pos_x" type="double" value="0.0" />
	<param name="wxx/initial_align/Init_Body_pos_y" type="double" value="0.0" />
	<param name="wxx/initial_align/Init_Body_pos_z" type="double" value="0.0" />
```

# 单独launch功能
> 下面分别介绍每个包各自的功能和相关的输入输出，方便第一次使用的时候检查是否因为topic没接对而跑不起来

> 同时大家使用时候会需要调整的主要参数。举个例子如果算力吃紧（经常表现为ekf报红错误，可以检查算法输出odom的频率是否和雷达点云频率一致来确认），则可以将下述算力相关的参数按照意思调整，如将```point_filter_num```变大。如果有更细致的要求可以直接联系作者王学习


## 1. **mapping_mid360.launch**
> 功能：
在**未知环境**下的雷达定位和建图，将保存一个可以拿来定位的地图new.pcd

* 输入：
    * livox雷达输出的原始点云: /livox/lidar
    * livox雷达输出的原始IMU: /livox/imu
* 输出：
    * 机体（不是雷达）的里程计：/Odometry
    * 机体（不是雷达）的位姿：/PoseStamped 
    * 世界坐标系下雷达扫到的点云：/cloud_registered

### 主要可调参数

* 算力相关（mapping_mid360.launch里）
```xml
<!-- 输入点云直接按照（计数%point_filter_num==0）降采样 -->
<param name="point_filter_num" type="int" value="3"/>
<!-- fastlio每次update里icp的迭代次数 -->
<param name="max_icp_times" type="int" value="2" />
<!-- fastlio每次update里总迭代次数（所有icp内部的迭代次数总和） -->
<param name="max_iteration" type="int" value="4" />
<!-- 输入点云的voxel降采样大小 -->
<param name="filter_size_surf" type="double" value="0.25" />
<!-- 建图点云的voxel降采样大小 -->
<param name="filter_size_map" type="double" value="0.5" />
```
* 常用功能（mid360.yaml）
```xml
<!-- 多少m内的点云直接扔掉 -->
preprocess/blind: 0.5
<!-- 这个ros包输出的odom和点云是机体坐标系的， 这里给出雷达关于机体的位姿接口 -->
<!-- Lidar_Odom = Lidar_wrt_Body_R*(Body_Odom + Lidar_wrt_Body_T) -->
Lidar_wrt_Body_R:
- 0.0000000
- 0.9661348
- 0.2580377
- -1.0000000
- 0.0000000
- 0.0000000
- 0.0000000
- -0.2580377
- 0.9661348
Lidar_wrt_Body_T:
- 0
- 0
- 0
```
## 2. **initial_align.launch**

> 功能：基于已知环境的点云地图origin.pcd和当前雷达静止时获得的点云进行GICP匹配，获得雷达在已知地图上的初始位置


* 输入：
    * livox雷达输出的原始点云: /livox/lidar
    * livox雷达输出的原始IMU(用于重力对齐以降低点云匹配的难度): /livox/imu
    * 已知环境的点云地图文件: origin.pcd
* 输出：
    * 机体（不是雷达）的初始里程计：/initial_odom_for_lio
    * 与已知环境匹配对齐后的点云，即雷达在已知地图的世界坐标系下的点云：/initial_cloud_from_odom

### 主要可调参数
```yaml
wxx:
  initial_align:
    voxelgrid_filter_size:  0.2 # 地图和输入点云降采样
    max_iteration:          10  # GICP次数
    icp_mode:               4   # 不同的ICP方法，测试4最好
    initial_map_size:       50  # 只取半径范围内的地图点云

    Init_Body_pos_x: 0.0
    Init_Body_pos_y: 0.0       # 初始位置的initial guess
    Init_Body_pos_z: 0.0
```




## 3. **odom_mid360_with_map.launch**
> 功能：
在**已知环境**的点云地图origin.pcd下进行定位（和incremental建图）

> **NOTE**： 必须接受到```initial_align.launch```算出的机体在已知点云下的初始位姿topoc : ```/initial_odom_for_lio```


* 输入：
    * livox雷达输出的原始点云: /livox/lidar
    * livox雷达输出的原始IMU: /livox/imu
* 输出：
    * 机体（不是雷达）的里程计：/Odometry
    * 机体（不是雷达）的位姿：/PoseStamped 
    * 世界坐标系下雷达扫到的点云：/cloud_registered


### 主要可调参数

* 上述**mapping_mid360.launch**的参数都在这里都有

* 其他常用功能（mid360.yaml）
```xml
<!-- 如果完全不需要增量建图，则可以将这个参数写为false -->
wxx/map_incremental: true
```




## 4. **fy_series_publisher.launch**
> 功能：
把从LIO发布出来的点云拆成合适的小包给FY地面站发布

* 输入：
    * 世界坐标系下雷达扫到的点云：/cloud_registered
    * 机体（不是雷达）的里程计：/Odometry
* 输出：
    * 拆成小包发布的点云：/fy_series_publisher/fy/cloud_fy



### 主要可调参数

```xml
<!-- 获取的雷达点云的频率 -->
<param name="lidar_freq" type="int" value="10"/>
<!-- 积累多少帧雷达点云后发布 -->
<param name="frame_pub_count" type="int" value="10"/>
<!-- 积累雷达点云过程中跳几帧 -->
<param name="frame_skip" type="int" value="0"/>
<!-- 对积累的点云降采样 -->
<param name="filter_size" type="double" value="0.2" />
<!-- 对积累的点云限高 -->
<param name="filter_ceil" type="double" value="2.5" />
<!-- 对积累的点云限底 -->
<param name="filter_floor" type="double" value="0.1" />
<!-- 对积累的点云限制距离 -->
<param name="filter_dist" type="double" value="20.0" />

<!-- 对积累的点云拆分成一个一个含filter_pc_size_each_pub数量点的小包 -->
<param name="filter_pc_size_each_pub" type="int" value="85" />
<!-- 对积累的点云拆分成小包的数量 -->
<param name="pub_freq" type="int" value="10" />
```
> 功能：
这个包的逻辑是在积累点云的```time = ((1 + frame_skip) * frame_pub_count / ( 1.0 * lidar_freq ))```时间内把积累的点进行各自滤波，然后拆成```pub_freq```个小包然后按照```pub_freq / time```的频率发布
