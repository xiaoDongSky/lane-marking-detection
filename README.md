![image](https://github.com/ZhihaiYan/figures/blob/master/vecan1.PNG)

Lane marking detection 
=========================


A ROS package for lane-marking detection in mutil-lanes scenarios.

[Vecan Lab](http://vecan.tongji.edu.cn/) | [Tongji](http://www.tongji.edu.cn/)

Performance
------------
* https://www.youtube.com/watch?v=b0HcHvPaSZE

Requirements
------------
* ROS操作系统（opencv，boost）
* 车载相机拍摄的数据（图像或者视频）



Installation
------------
```
git clone https://github.com/ZhihaiYan/lane-marking-detection.git
```

Quick Start
-------------
* 配置ROS环境（kinetic）
* 将车载相机录制的图像数据放到指定文件夹下
* 按照实际数据的格式修改launch文件
* 运行即可得到检测结果

Parameters
-------------
* Lane width: 170 pixels
* Max lanes：6
* Max deteciton distance: 50m


Discussing
----------
- email: zhyan@tongji.edu.cn
