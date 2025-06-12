
#include "clusters.h"



void KdTree::insert(pcl::PointXYZ point, int id)
{
    insertHelper(&root,point,id,0);
};

void KdTree::insertHelper(Node** node, pcl::PointXYZ point, int id, int depth)
{
    if(*node == NULL)
    {
        *node = new Node(point,id);
        return;
    }

    int cd = depth % 3;

    if(point.data[cd] <= (*node)->point.data[cd])
        insertHelper(&((*node)->left),point,id,++depth);
    else
        insertHelper(&((*node)->right),point,id,++depth);

}
// return a list of point ids in the tree that are within distance of target
std::vector<int> KdTree::search(pcl::PointXYZ target, float distanceTol)
{
    std::vector<int> ids;
    searchHelper(target,distanceTol,root,ids,0);
    return ids;
}

void KdTree::searchHelper(pcl::PointXYZ target, float distanceTol, Node *node, std::vector<int>& ids,int depth)
{
    if(node != NULL){
        
        if((node->point.data[0]>=(target.x-distanceTol)) && (node->point.data[0] <= (target.x+distanceTol))
         && (node->point.data[1] >= (target.y - distanceTol)) &&(node->point.data[1]<=(target.y + distanceTol))
         && (node->point.data[2] >= (target.z- distanceTol)) && (node->point.data[2]<=(target.z + distanceTol)))
        {
            float distance = sqrt((node->point.data[0]-target.x) * (node->point.data[0]-target.x) + (node->point.data[1]- target.y)*(node->point.data[1]-target.y) + (node->point.data[2]-target.z)*(node->point.data[2]-target.z));
            if(distance <= distanceTol)
                ids.push_back(node->id);
        }

        if((target.data[depth%3] - distanceTol) < node->point.data[depth%3])
        {
            searchHelper(target,distanceTol,node->left,ids,++depth);
        }
        if((target.data[depth%3] + distanceTol) > node->point.data[depth%3])
        {
            searchHelper(target,distanceTol,node->right,ids,++depth);
        }
    }
}




