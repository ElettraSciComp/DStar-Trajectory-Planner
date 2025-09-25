# [D* Dynamic Trajectory Planner](https://wiki.ros.org/dstar_trajectory_planner)

`move_base` plugin implementing global trajectory planning with [D* informed incremental search algorithm](https://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.15.3683) with external optimizers. It strives to simplify development of the robotic systems operating in dense industrial environments.

## Overview

The package contains two plugins:
- **D-star trajectory planner** that maintains generation of the global trajectory on the provided costmap, and dynamic replanning by request coming from `move_base`.
- **Virtual Walls module** processing JSON objects to generate the non-passable and explicitly passable zones as OpenCV polygons added over the existing map.

## Parameters
The plugin accepts the following parameters

|Parameter|Unit|Default value|Description|
|---------|----|-------------|-----------|
|`goal_distance_threshold`| m |0.3|Maximal distance from the chassis' reference frame origin to consider the goal reached|
|`neighbor_distance_threshold`| m |0.1|The distance to consider as close for replanning|
|`occupancy_threshold`|-|64|`OccupancyGrid` cell weight threshold to consider the cell non-passable|
|`cutoff_distance`|`OccupancyGrid` cells|16|Maximal cutoff distance for raytracing optimizer|
|`trajectory_optimizer`|-|-|Group of parameters to set up the potential field optimizer|
|`trajectory_optimizer/repulsion_gain`|-|50.0|Gain value for repulsive potential calculation|
|`trajectory_optimizer/potential_field_radius`|`OccupancyGrid` cells|10|Maximal repulsion radius to calculate the potential during optimization|
|`erosion`|-|-|Group of parameters to set up map preprocessor|
|`erosion/enable`|-|`false`|Applies an erosion algorithm to clean a noisy map|
|`erosion/erosion_gap`|`OccupancyGrid` cells|2|Erosion gap|

## Development and Roadmap
The plugin was developed within the Robotics and Remotization initiative held at Elettra Sincrotrone Trieste. It is a part of an innovative flexible control system for mobile robots that was presented by the group leaders in Tokyo during 16th [IFToMM](https://iftomm-world.org) [World Congress](https://wc2023.jc-iftomm.org/) 2023. The interested ones can find the published paper here: https://doi.org/10.1007/978-3-031-45770-8_29
