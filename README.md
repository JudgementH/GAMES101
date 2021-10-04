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

