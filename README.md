# GAMES101
Course assignments for GAMES101



# Assignment 0 （a0）环境配置

操作系统：Windows 10

编译器：MinGW-x86_64-posix

IDE：Clion



对于Eigen的安装可以参考文章

[Windows10下配置VSCode、Mingw、Cmake、Eigen、OpenCV环境（为GAMES101作业构筑Win环境）](https://zhuanlan.zhihu.com/p/363769672)



**关于CMakeLists**

在导包时，需要把`include_directories(EIGEN3_INCLUDE_DIR)`

修改为`include_directories("C:/Program Files (x86)/Eigen3/include")`（这里给出的是Eigen默认安装位置，可根据自己需要调整）



# Assignment 1（a1）旋转、投影



## 关于CMakeLists

导包opencv，同样参考a0文章进行opencv的安装

修改`include_directories(/usr/local/include)`，

添加自己的opencv安装位置如`include_directories("E:/include/opencv-cpp/opencv/sources/build/install")`



导包eigen

需要添加`find_package(Eigen3 REQUIRED)`

同时添加`include_directories("C:/Program Files (x86)/Eigen3/include")`



## 结果

![image-20211005131053984](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20211005131053984.png)