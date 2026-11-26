## Limo packages

This repo contains five packages that exploit the LIMO's visual information to build **global/local costmaps** and significantly improve the accuracy of **autonomous localization**. The packages are compatible with **ROS 2 Foxy**.

#### 1. limo_depth

Assuming the LIMO travels on a **planar surface**, this package estimates the pose of the traveling plane through frame sampling and uses it to estimate the values of the **unknown pixels** in the depth image.

#### 2. limo_cloud

This package takes **labeled grayscale and depth images** as input and transforms them into **planar clouds of classified points**.

#### 3. limo_cloud_utilities

This package provides **plugins for `limo_cloud`** that help classify the visual obstacles of the main circuit at CRAN. The inputs are a grayscale image and a depth image of partially labeled data, i.e. the road, special classes such as yellow lines, and a general background class. The package includes functions that **subsample both images and point clouds** to speed up downstream computation, as well as a function that helps distinguish the **white road lines** from the **sidewalk**.

#### 4. limo_map

This package takes as input the lidar data and a point cloud with the same properties as the output of `limo_cloud`, and builds a **combined costmap** of the circuit's obstacles. The launch files also start a node that uses the lidar data for **SLAM**.

It produces three outputs:

- a **combined map**, usable for trajectory planning and control;
- a **lidar-only obstacle map**;
- a map containing only the **main CV obstacles**.

The last two maps are created as a reference for autonomous localization.

#### 5. limo_amcl

This package modifies the **AMCL of the Nav2 stack** so that it also uses the robot's **visual information**. The resulting covariance is much lower than the one obtained with the classical procedure.
