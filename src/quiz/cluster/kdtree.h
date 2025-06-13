/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
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


	void insert( std::vector<float> point, int id)
	{
		insertHelper(&root,point,id,0);
	}

	void insertHelper(Node** node, std::vector<float>point, int id, int depth)
	{
		if(*node == NULL)
		{
			*node = new Node(point,id);
			return;
		}

		int cd = depth % 3;

		if(point[cd] <= (*node)->point[cd])
			insertHelper(&((*node)->left),point,id,++depth);
		else
			insertHelper(&((*node)->right),point,id,++depth);

	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target,distanceTol,root,ids,0);
		return ids;
	}

	void searchHelper(std::vector<float> target, float distanceTol, Node *node, std::vector<int>& ids,int depth)
	{
		if(node != NULL){
			
			if((node->point[0]>=(target[0]-distanceTol)) && (node->point[0] <= (target[0]+distanceTol)) && (node->point[1] >= (target[1] - distanceTol)) &&(node->point[1]<=(target[1] + distanceTol))
			&& (node->point[2]>=(target[2]-distanceTol)) && (node->point[2] <= (target[2]+distanceTol)))
			{
				float distance = sqrt((node->point[0]-target[0]) * (node->point[0]-target[0]) + (node->point[1]- target[1])*(node->point[1]-target[1])+ (node->point[2]- target[2])*(node->point[2]-target[2]));
				if(distance <= distanceTol)
					ids.push_back(node->id);
			}

			if((target[depth%3] - distanceTol) < node->point[depth%3])
			{
				searchHelper(target,distanceTol,node->left,ids,++depth);
			}
			if((target[depth%3] + distanceTol) > node->point[depth%3])
			{
				searchHelper(target,distanceTol,node->right,ids,++depth);
			}
		}
	}
};




