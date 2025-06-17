#ifndef RANSAC3D_H_
#define RANSAC3D_H_
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);
#endif