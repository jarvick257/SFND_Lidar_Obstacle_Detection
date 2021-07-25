#pragma once

namespace project {

template<typename PointT>
class Node
{
  public:
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT p, int setId)
	:	point(p), id(setId), left(nullptr), right(nullptr)
	{}

	~Node() {
		delete left;
		delete right;
	}
};

template<typename PointT>
class KdTree
{
  public:
	KdTree() : root(nullptr) {}
	~KdTree() {
		delete root;
	}

	void insert(const PointT& point, int id)
	{
        if(nullptr == root){
                root = new Node<PointT>(point, id);
                return;
        }

        Node<PointT>* node = root;
        int lvl = 0;
        while(true){
            Node<PointT>** child; 
            switch(lvl % 3){
                case 0:
                    child = (point.x < node->point.x) ?  &node->left : &node->right;
                    break;
                case 1:
                    child = (point.y < node->point.y) ?  &node->left : &node->right;
                    break;
                case 2:
                    child = (point.z < node->point.z) ?  &node->left : &node->right;
                    break;
            }
            if(*child == nullptr){
                *child = new Node<PointT>(point, id);
                return;
            }
            node = *child;
            lvl++;
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }

  private:
	Node<PointT>* root;
    void searchHelper(PointT target, Node<PointT>*  node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if(nullptr == node){
            return;
        }

        if(node->point.x >= target.x - distanceTol &&
                node->point.x <= target.x + distanceTol &&
                node->point.y >= target.y - distanceTol &&
                node->point.y <= target.y + distanceTol &&
                node->point.z >= target.z - distanceTol &&
                node->point.z <= target.z + distanceTol ){
            const float dx = node->point.x - target.x;
            const float dy = node->point.y - target.y;
            const float dz = node->point.z - target.z;
            const float distance = sqrt(dx*dx + dy*dy + dz*dz);
            if(distance <= distanceTol){
                ids.push_back(node->id);
            }
        }

        bool checkLeft = false;
        bool checkRight = false;
        switch(depth % 3){
            case 0:
                checkLeft = target.x - distanceTol < node->point.x;
                checkRight = target.x + distanceTol > node->point.x;
                break;
            case 1:
                checkLeft = target.y - distanceTol < node->point.y;
                checkRight = target.y + distanceTol > node->point.y;
                break;
            case 2:
                checkLeft = target.z - distanceTol < node->point.z;
                checkRight = target.z + distanceTol > node->point.z;
                break;
        }
        if(checkLeft){
            searchHelper(target, node->left, depth+1, distanceTol, ids);
        }
        if(checkRight){
            searchHelper(target, node->right, depth+1, distanceTol, ids);
        }
    }

};
}
