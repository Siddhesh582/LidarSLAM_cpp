# LIDAR SLAM 

C++ LiDAR SLAM project.

## Structure

- `simulator/` - motion models (Unicycle, NoisyUnicycle)
- `ICP.h`, `ICP.cpp` - Iterative Closest Point registration

## Build
```
g++ -std=c++20 -T /usr/include/eigen3 -c simulator/Unicycle.cpp
```

