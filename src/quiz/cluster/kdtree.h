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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
      insertHelper(root, point, id);
	}
  
  void insertHelper(Node *&node, std::vector<float> point, int id, int layer = 0) {
      if(node == NULL)
      {
        node = new Node(point, id);
        std::cout << "current node ID: " << node->id;
        std::cout << " point(" << node->point[0] << ", " << node->point[1] << ")"<< std::endl;
      }
      else if(point[layer%2] < node->point[layer%2])
      {
        std::cout << "Layer" << layer << " Left->";
        insertHelper(node->left, point, id, ++layer);
      }
      else
      {
        std::cout << "Layer" << layer << " Right->";
        insertHelper(node->right, point, id, ++layer);
      }
  }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
      	searchHelper(root, target, distanceTol, ids);
		return ids;
	}
	
  void searchHelper(Node* node, std::vector<float> target, float distanceTol, std::vector<int>& ids, int layer = 0) {
    if(node == NULL) return;
    float dx = std::fabs(target[0] - node->point[0]);
    float dy = std::fabs(target[1] - node->point[1]);
    if(sqrt(dx*dx + dy*dy) <= distanceTol) {
      ids.push_back(node->id);
    } 
    if(target[layer%2] - distanceTol <= node->point[layer%2]) {
      searchHelper(node->left, target, distanceTol, ids, ++layer);
    }
    if(target[layer%2] + distanceTol >= node->point[layer%2]) {
      searchHelper(node->right, target, distanceTol, ids, ++layer);
    }
  }
    
};




