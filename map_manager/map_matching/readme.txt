/* map matching between two file.txt composed by features:

- angles_mod.txt : features coming from odometry analysis of a graph-represented map (rovina code: catacomb.slam.log analysis with pwn_slam_map_reading) (the '_mod version'is manually builded checking and deleting double almost coincident point features)

- inters_point_manual.txt : features coming by the manual (pixel) selecting using a (i.e.).png image file coming from the image processing done with distance_map bin plus matlab routine to identify the skeleton intersection of the planimetry corridors

/* adding new Poses to the graph as gaussian priors (Unary edge):
- using the pwn_app (log file) format: the g2o file is a graph SE3 (with no measurements).
