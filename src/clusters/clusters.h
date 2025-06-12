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
    pcl::PointXYZ point;
    Node* left;
    Node* right;

    Node(pcl::PointXYZ arr, int setId)
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


	void insert(pcl::PointXYZ point, int id);
	void insertHelper(Node** node, pcl::PointXYZ point, int id, int depth);
	std::vector<int> search(pcl::PointXYZ target, float distanceTol);
	void searchHelper(pcl::PointXYZ target, float distanceTol, Node *node, std::vector<int>& ids,int depth);
};

#endif