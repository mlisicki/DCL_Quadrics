DCL_Quadrics - DisCODe Component Library
========================================

Description
-----------

Performs quadric segmentation based on depth and normal maps labelling (onNewImage) and point cloud (onNewPointCloud). Based on RANSAC matching. On output gives colored segments and symbolic PROLOG like description of quadrics in given scene.

Parameters:

> label_size - threshold for number of points within one region to consider this region for further processing (performance issue)

> inliers_size - threshold for number of inliers after RANSAC matching of region to consider the region as matched to model

> t_distance - threshold of distance between points in a cloud for RANSAC algorithm

Dependencies
------------

Depth, DCL_CameraNUI

Maintainer
----------

Michal Lisicki
