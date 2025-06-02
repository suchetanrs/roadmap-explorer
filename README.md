## RoadMap-Explorer: A fast and reliable robot exploration framework

This repository contains the complete implementation of the exploration algorithm for UGVs that I've developed over the course of the last 15 months.

## Citations:
If you consider this work was useful to you, please consider citing the following which contain the theoretical background of the algorithm.

> S. Saravanan, A. Bains, C. Chanel, D. Vivet: "FIT-SLAM 2: Efficient 3d Exploration with Fisher Information and Traversability Based Adaptive Roadmap"

## Introduction:

Roadmap explorer contains an exploration algorithm that primarily builds on top of the frontier exploration approach, but with a lot more ofcourse :)

This exploration framework primarily provides the following key advantages when compared to other frameworks out there.
1. End to end support to run with Nav2 (including Nav2 lifecycle and a behavior tree plugin to run with your custom behavior trees!)
2. An exploration framework that scales well with the size of the map: successful exploration sessions have been recorded upto 3,520 sq.m (~37,000 sq.ft). However, I don't believe that's the limit. Since the Frontier search and the TSP solvers are limited to a specified radius, it can potentially explore endlessly provided the SLAM supports it.
3. Something that's efficient: The average computational load on a standard platform such as the Jetson Xavier is limited to 10% of a single core of the CPU during regular exploration and spikes of upto 20-25% during global repositioning.
4. Utility features: There are a few important features that are included as a part of this package. Namely:
    - Ability to save exploration sessions (explored map as well as the constructed roadmap) in between runs and to continue exploring from where you left off last time.
    - Explore in localisation only mode: In certain use cases, when the robot is running in localisation only mode, it might still need to explore the envioronment. For example, to find new objects to pickup, clean and tidy or to re-explore the area with a secondary sensor on board such as a camera to detect tags / visual landmarks. This is well supported within this framework with the ability to configure different FOVs.
    - An RViz plugin that can be used to command the exploration server with your preferred method as well as to save and load past sessions.
5. (Probably the most important): The result is a much more structured exploration than most other popular frameworks I've tried out there. As a result, the total time taken to explore the environment is much lower. Roughly 25% faster than [GB-Planner 2](https://github.com/ntnu-arl/gbplanner_ros), 45% faster than [NBVP](https://github.com/ethz-asl/nbvplanner) or [frontier based approaches](https://github.com/paulbovbel/frontier_exploration) that are greedy in nature and 30% imporovement when comparted to approaches that couple arrival utility with path length [FIT-SLAM](https://ieeexplore.ieee.org/document/10553174).