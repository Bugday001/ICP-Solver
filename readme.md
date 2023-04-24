# CERES-ICP
使用CERES库进行icp匹配

目前支持两种计算方法：解析求导和自动求导。
通过yaml中的is_autoDiff选择。

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