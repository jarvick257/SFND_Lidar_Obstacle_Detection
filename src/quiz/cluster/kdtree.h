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

	void insert(std::vector<float> point, int id)
	{
        if(nullptr == root){
                root = new Node(point, id);
                return;
        }

        Node* node = root;
        int lvl = 0;
        while(true){
            const int dim = lvl % point.size();
            Node** child = (point[dim] < node->point[dim]) ?  &node->left : &node->right;
            if(*child == nullptr){
                *child = new Node(point, id);
                return;
            }
            node = *child;
            lvl++;
        }
	}

    void searchHelper(std::vector<float> target, Node*  node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if(nullptr == node){
            return;
        }

        if(node->point[0] >= target[0] - distanceTol &&
           node->point[0] <= target[0] + distanceTol &&
           node->point[1] >= target[1] - distanceTol &&
           node->point[1] <= target[1] + distanceTol ){
            const float dx = node->point[0] - target[0];
            const float dy = node->point[1] - target[1];
            const float distance = sqrt(dx*dx + dy*dy);
            if(distance <= distanceTol){
                ids.push_back(node->id);
            }
        }

        const int dim = depth % target.size();
        if(target[dim] - distanceTol < node->point[dim]){
            searchHelper(target, node->left, depth+1, distanceTol, ids);
        }
        if(target[dim] + distanceTol > node->point[dim]){
            searchHelper(target, node->right, depth+1, distanceTol, ids);
        }

    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




