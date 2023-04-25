# CERES-ICP
使用CERES库进行icp匹配

GICP：支持自动求导。使用llt()分解似乎效果还不如直接相乘。

Point-to-Point ICP：支持解析求导和自动求导。通过yaml中的is_autoDiff选择。

支持Debug输出每次运行时间，支持设置ceres使用多线程。
## TODO
- [X] Plane-to-Plane ICP.(GICP)。
- [ ] Using nanoFLANN to speed up.
- [ ] Is there any way to optimize the way when creating a ceres solver.

## Acknowledgements
Thanks for [testICP](https://github.com/chengwei0427/testICP), 
[ALOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM),
[gicp](https://github.com/avsegal/gicp) 
and 
[ceres blog](https://blog.csdn.net/qq_42911741/article/details/127326164)