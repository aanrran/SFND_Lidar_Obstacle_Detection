#include "cluster3D.h"


template<typename PointT>
void KdTree3D<PointT>::Proximity(std::vector<std::pair<std::vector<float>, bool>>& pairedPoints, std::vector<int>& cluster, int idx) {
  //mark point as processed
  pairedPoints[idx].second = true;
  //add point to cluster
  cluster.push_back(idx);
  //nearby points = tree(point)
  std::vector<int> points = this->search(pairedPoints[idx].first);
  //Iterate through each nearby point
  for(int pointIdx : points) {
    //If point has not been processed
    //Proximity(cluster)
    if(pairedPoints[pointIdx].second == false) Proximity(pairedPoints, cluster, pointIdx);
  }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> KdTree3D<PointT>::euclideanCluster()
{
  // TODO: Fill out this function to return list of indices for each cluster
  std::vector<std::pair<std::vector<float>, bool>> pairedPoints;
  //mark the point to false - has not clustered
  for(PointT point : cloud->points) {
    std::vector<float> pointXYZ(3);
    pointXYZ[0] = point.x;
    pointXYZ[1] = point.y;
    pointXYZ[2] = point.z;

    pairedPoints.push_back(std::make_pair(pointXYZ, false));
  }
  //create the Ktree
  for(int i = 0; i < pairedPoints.size(); i++) this->insert(pairedPoints[i].first, i);
  
  //create the clusters
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  for(int idx = 0; idx < cloud->points.size(); idx++) {
    if(pairedPoints[idx].second == false) {
      std::vector<int> cluster;
      Proximity(pairedPoints, cluster, idx);
      // check if the cluster size is within the range
      if(cluster.size() >= this->minsize && cluster.size() <= this->maxsize) {
        //add cluster points into a clustered cloud
     	typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for(int indice: cluster) clusterCloud->points.push_back(cloud->points[indice]);
        clusters.push_back(clusterCloud);
      }
    }
  }
  
  return clusters;

}
template<typename PointT>
void KdTree3D<PointT>::insert(std::vector<float> point, int id)
{
  // TODO: Fill in this function to insert a new point into the tree
  // the function should create a new node and place correctly with in the root
  insertHelper(this->root, point, id);
}
template<typename PointT>
void KdTree3D<PointT>::insertHelper(Node *&node, std::vector<float> point, int id, int layer) {
//if the branch is empty, add node
  if(node == NULL) node = new Node(point, id);
  //if the value is less than the parents point, add this child point to its left
  else if(point[layer%3] < node->point[layer%3]) insertHelper(node->left, point, id, ++layer);
  //otherwise to the right
  else insertHelper(node->right, point, id, ++layer);
}

// return a list of point ids in the tree that are within distance of target
template<typename PointT>
std::vector<int> KdTree3D<PointT>::search(std::vector<float> target)
{
  std::vector<int> ids;
  searchHelper(this->root, target, ids);
  return ids;
}
template<typename PointT>
void KdTree3D<PointT>::searchHelper(Node* node, std::vector<float> target, std::vector<int>& ids, int layer) {
  if(node == NULL) return;
  float dx = std::fabs(target[0] - node->point[0]);
  float dy = std::fabs(target[1] - node->point[1]);
  float dz = std::fabs(target[2] - node->point[2]);
  //if the point is nearby, then add point to the ids list
  if(sqrt(dx*dx + dy*dy + dz*dz) <= this->distanceTol) {
    ids.push_back(node->id);
  } 
  //check the point is within the left bound, if so search left
  if(target[layer%3] - this->distanceTol <= node->point[layer%3]) {
    searchHelper(node->left, target, ids, ++layer);
  }
  //check the point is within the right bound, if so search right
  if(target[layer%3] + this->distanceTol >= node->point[layer%3]) {
    searchHelper(node->right, target, ids, ++layer);
  }
}
