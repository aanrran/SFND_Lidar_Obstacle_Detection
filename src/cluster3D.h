// PCL lib Functions for processing point clouds 

#ifndef CLUSTER3D_H_
#define CLUSTER3D_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <math.h>

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

template<typename PointT>
class KdTree3D
{
	public:
	//constructor
	KdTree3D(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, float minsize, float maxsize):cloud(cloud), distanceTol(distanceTol), minsize(minsize), maxsize(maxsize), root(NULL) {}
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster();
    
    private:
    //private functions
    void Proximity(std::vector<std::pair<std::vector<float>, bool>>& pairedPoints, std::vector<int>& cluster, int idx);
    void insert(std::vector<float> point, int id);
    void insertHelper(Node *&node, std::vector<float> point, int id, int layer = 0);
    std::vector<int> search(std::vector<float> target);
    void searchHelper(Node* node, std::vector<float> target, std::vector<int>& ids, int layer = 0);
    
    //private variables
    const typename pcl::PointCloud<PointT>::Ptr cloud;
    const float distanceTol;
    Node* root;
    const float maxsize;
    const float minsize;
};

#endif


