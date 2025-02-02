# apf_local_planner

The apf_local_planner package, modified from [local_planner](https://github.com/amov-lab/Prometheus/tree/v1.1/Modules/planning/local_planner)

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FAPF-Planner.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-noetic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/MATLAB-2023b-?logo=)
![Static Badge](https://img.shields.io/badge/Ubuntu-20.04.6-E95420?logo=ubuntu)

video on [bilibili](https://www.bilibili.com/video/BV1Lr421u7z9/) 

## Release Note

- v1.2.0: support `XYZ_VEL` control
- v1.1.0: drone facing front while moving

```bash
catkin_make --source plan/apf_local_planner --build plan/apf_local_planner/build
```

```bash
roslaunch apf_local_planner simulation.launch
```

![rqt_graph](log/2024-05-08/rosgraph.png)
