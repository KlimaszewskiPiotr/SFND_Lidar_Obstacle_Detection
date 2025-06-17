#ifndef CLUSTERS_H_
#define CLUSTERS_H_
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <vector>

struct Node
{
    int id;
    pcl::PointXYZI point;
    Node* left;
    Node* right;

    Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}

};
struct KdTree
{
    Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}


	void insert(pcl::PointXYZI point, int id);
	void insertHelper(Node** node, pcl::PointXYZI point, int id, int depth);
	std::vector<int> search(pcl::PointXYZI target, float distanceTol);
	void searchHelper(std::vector<int>& ids, pcl::PointXYZI target,float distance, Node* node, int depth);
};

std::vector<std::vector<int>> euclideanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, typename KdTree* tree, float distanceTo, int min, int max);
void proximity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processedPoints, int id, typename KdTree* tree, float distanceTo);

#endif