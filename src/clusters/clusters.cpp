#include "clusters.h"




void KdTree::insert(pcl::PointXYZ point, int id)
{
    insertHelper(&root,point,id,0);
}

void KdTree::insertHelper(Node** node,pcl::PointXYZ point,int id,int depth)
{
    if(*node == NULL)
    {
        *node = new Node(point, id);
        return;
    }

    int cd = depth%3;

    if(point.data[cd] <= (*node)->point.data[cd])
        insertHelper((&(*node)->left),point,id,++depth);
    else
        insertHelper((&(*node)->right),point,id,++depth);

};

std::vector<int> KdTree::search(pcl::PointXYZ target, float distance)
{
    std::vector<int> ids;
    searchHelper(ids, target,distance, root,0);
    return ids;
}

void KdTree::searchHelper(std::vector<int>& ids, pcl::PointXYZ target,float distance, Node* node, int depth)
{
    //std::cout << "Search Helper function goes with: " << distance << depth << "\n";
    if(node!=NULL)
    {
        std::cout << "Node position (" << node->point.x << ", " << node->point.y << ", " << node->point.z << ")\n";
        std::cout << "Target position (" << target.x << ", " << target.y << ", " << target.z << ")\n"; 

        //if(true)
        if((node->point.x >= (target.x - distance)) && (node->point.x <= (target.x + distance))
        && (node->point.y >= (target.y - distance)) && (node->point.y <= (target.y + distance))
        && (node->point.z >= (target.z - distance)) && (node->point.z <= (target.z + distance)))
        {
            float distanceTo = sqrt((node->point.x-target.x)*(node->point.x-target.x)
            +(node->point.y-target.y)*(node->point.y - target.y)
            +(node->point.z-target.z)*(node->point.z-target.z));

           std::cout << "Calculated distance " << distanceTo << "\n";

            if(distanceTo <= distance)
            {
                ids.push_back(node->id);
                std::cout << "Added point to nearby poitn\n";
            }
        }

        if((target.data[depth%3]-distance) < node->point.data[depth%3])
            searchHelper(ids,target,distance,node->left,++depth);
        
        if((target.data[depth%3] + distance) > node->point.data[depth%3])
            searchHelper(ids,target,distance,node->right,++depth);
    }
}

std::vector<std::vector<int>> euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,KdTree* tree, float distanceTo, int min, int max)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processedPoint(cloud->points.size(),false);
    std::cout << "Started euclideaClustering with " << cloud->points.size() << " amount of points\n";
    for(int i = 0;i<cloud->points.size();++i)
    {
        if(processedPoint[i] == false)
        {
            std::cout << "Adding new cluster\n";
            std::vector<int> cluster;
            proximity(cloud,cluster,processedPoint,i,tree,distanceTo);
            if(cluster.size()>= min && cluster.size()<=max)
                clusters.push_back(cluster);
        }
    }
    return clusters;
}

void proximity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processedPoints, int id, KdTree* tree, float distanceTo)
{
    std::cout << "Activated Proximity for point " << id << "\n";
    processedPoints[id] = true;
    cluster.push_back(id);
    std::vector<int>nearbyPoints = tree->search(cloud->points[id],distanceTo);
    std::cout << "Nearby Points size: " << nearbyPoints.size()  << "\n";
    for(int index = 0;index < nearbyPoints.size();++index)
    {
        if(processedPoints[nearbyPoints[index]] == false)
            proximity(cloud,cluster,processedPoints,nearbyPoints[index],tree,distanceTo);
    }
}




