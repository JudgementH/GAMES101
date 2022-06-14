# GAMES101
Course assignments for GAMES101



作业的初始源代码可以在[GAMES论坛](https://games-cn.org/forums/topic/allhw/)中找到



# Assignment 0 (a0) 环境配置

操作系统：Windows 10

编译器：MinGW-x86_64-posix

IDE：Clion



对于Eigen的安装可以参考文章

[Windows10下配置VSCode、Mingw、Cmake、Eigen、OpenCV环境（为GAMES101作业构筑Win环境）](https://zhuanlan.zhihu.com/p/363769672)



**关于CMakeLists**

在导包时，需要把`include_directories(EIGEN3_INCLUDE_DIR)`

修改为`include_directories("C:/Program Files (x86)/Eigen3/include")`（这里给出的是Eigen默认安装位置，可根据自己需要调整）



**更便捷的安装方式**

可以尝试使用微软的vcpkg（cpp库管理工具，类似pip）进行库的安装



# Assignment 1 (a1) 旋转、投影



## 关于CMakeLists

导包opencv，同样参考a0文章进行opencv的安装

修改`include_directories(/usr/local/include)`，

添加自己的opencv安装位置如`include_directories("E:/include/opencv-cpp/opencv/sources/build/install")`



导包eigen

需要添加`find_package(Eigen3 REQUIRED)`

同时添加`include_directories("C:/Program Files (x86)/Eigen3/include")`



## 结果

![image-20211005131053984](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20211005131053984.png)



## 倒三角问题

得到的三角形和理论描述不一样，是因为**三角形的z坐标都是负值，但照相机是在（0，0，0）并且看向z轴的正方向，即三角形在照相机的后面**。根据projection投影的推导，相当于是倒立的，这和小孔成像倒立的原因是一样的。



## 如何修正倒三角？

可以把照相机参数中近平面和远平面取负值。

```c++
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
    zNear = -zNear;	//取负值
    zFar = -zFar;	//取负值
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    return projection;
}
```

结果可以恢复正常。

![image-20220604125423511](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220604125424680.png)



# Assignment 2 (a2) 三角形、z-buffer



## 结果

![image-20220604131328379](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220604131328379.png)



## SSAA2x

![image-20220606022913722](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220606022913722.png)



## MSAA2x

![image-20220606103033990](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220606103033990.png)



# Assignment 3 (a3) Shading



## Phong

![image-20220607140920214](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220607140920392.png)



## Texture

![image-20220607140541425](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220607140541425.png)



## Bump

![image-20220613002949369](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220613002949369.png)



## Displacement

![image-20220613003707284](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220613003708919.png)



## 其他模型

![image-20220613005646540](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220613005646540.png)



## 双线性纹理插值



**无插值**

<img src="https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220613114728259.png" alt="image-20220613114728259" style="zoom:200%;" />



**插值**

<img src="https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220613121533029.png" alt="image-20220613121533029" style="zoom: 200%;" />



# Assignment 4 (a4) 贝塞尔曲线

## 结果

![image-20220613175801277](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220613175801277.png)

![image-20220613175915947](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220613175915947.png)



## 反走样

![image-20220613190249612](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220613190249612.png)





# Assignment 5 (a5) 光线与三角形相交

## 结果

![image-20220614140858972](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220614140858972.png)



# Assignment 6 (a6) 光线追踪、加速结构、BVH、SVH



## 结果

![image-20220614205201157](https://cdn.jsdelivr.net/gh/JudgementH/image-host/md/image-20220614205201157.png)

用时7s



## SAH

改为BVH使用SAH划分后，仍为7s加速不太明显。