/* \author Aaron Brown */
// Quiz on implementing kd tree

//#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	: point(arr), id(setId), left(NULL), right(NULL){}
};

struct KdTree
{
	Node* root;

	KdTree(): root(NULL){}

	void insert(std::vector<float> point, int id){
		recursive_insert(&root, 0, point, id);
	}

	void recursive_insert(Node **root, int depth, std::vector<float>  point, int id){
	    if (*root!= NULL){
	        int i = depth%2;
	        if(point[i] < (*root)->point[i]){
	            // left
	            recursive_insert(&(*root)->left, depth+1, point, id);
	        } else{
	            //right
	            recursive_insert(&(*root)->right, depth+1, point, id);
	        }
	    }else{
            *root = new Node(point, id);
	    }
	}

	void recursive_search(Node * node, int depth, std::vector<int> &ids,
                       std::vector<float> target, float distanceTol){
	    if(node != NULL){
	        // compare current node to target
	        if ((node->point[0] >= (target[0]-distanceTol)) && (node->point[0] <= (target[0]+distanceTol)) &&
                    (node->point[1] >= (target[1]-distanceTol)) && (node->point[1] <= (target[1]+distanceTol))){

	            float dis = sqrt((node->point[0]-target[0]) * (node->point[0]-target[0]) +
	                    (node->point[1]-target[1]) * (node->point[1]-target[1]));

	            if (dis <= distanceTol){
	                ids.push_back(node->id);
	            }
	        }
	        if((target[depth%2] - distanceTol)<node->point[depth%2]){
	            // go to left
                recursive_search(node->left, depth + 1, ids, target, distanceTol);
            }
            if((target[depth%2] + distanceTol)>node->point[depth%2]){
                // go to right
                recursive_search(node->right, depth+1, ids, target, distanceTol);
            }
	    }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol){
		std::vector<int> ids;
		recursive_search(root, 0, ids, target, distanceTol);
		return ids;
	}
};




