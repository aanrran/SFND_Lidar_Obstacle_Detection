// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


//---------------------------my code here---------------------------------
//segmentation RANSAC Algorithm
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
  for(int interation = 0; interation < maxIterations; interation++) { 
    // Randomly sample subset and fit line
    int cloudDataLen = cloud->width;
    int randomID1, randomID2, randomID3;
    randomID1 = randomID2 = randomID3 = rand() % cloudDataLen;
    while(randomID1 == randomID2 || randomID1 == randomID3 || randomID2 == randomID3) {
      randomID2 = rand() % cloudDataLen;
      randomID3 = rand() % cloudDataLen;
    }

    PointT pt1 = cloud->points[randomID1];
    PointT pt2 = cloud->points[randomID2];
    PointT pt3 = cloud->points[randomID3];
	// Measure distance between every point and fitted line
    double A = (pt2.y - pt1.y)*(pt3.z - pt1.z) - (pt2.z - pt1.z)*(pt3.y - pt1.y);
    double B = (pt2.z - pt1.z)*(pt3.x - pt1.x) - (pt2.x - pt1.x)*(pt3.z - pt1.z);
    double C = (pt2.x - pt1.x)*(pt3.y - pt1.y) - (pt2.y - pt1.y)*(pt3.x - pt1.x);
    double D = -(A*pt1.x + B*pt1.y + C*pt1.z);
	// If distance is smaller than threshold count it as inlier
    std::unordered_set<int> tempInliersResult;
    for(int i = 0; i < cloudDataLen; i++) {
      PointT pt = cloud->points[i];
      double d = std::fabs(A*pt.x + B*pt.y + C*pt.z + D)/sqrt(A*A + B*B + C*C);
      if(d <= distanceTol) tempInliersResult.insert(i);
    }
  
	// Return indicies of inliers from fitted line with most inliers
	if(tempInliersResult.size() > inliersResult.size()) inliersResult = tempInliersResult;
  }

	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SplitClouds(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) 
{
  // TODO: Change the max iteration and distance tolerance arguments for Ransac function
  std::unordered_set<int> inliers = ProcessPointClouds<PointT>::Ransac3D(cloud, maxIterations, distanceThreshold);
  std::cout<< "Cloud total amount: " << cloud->points.size() << std::endl;
  std::cout<< "inliers total amount: " << inliers.size() << std::endl;
  typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());
  //add inlier and outlier point to the cloud
  for(int index = 0; index < cloud->points.size(); index++)
  {
    PointT point = cloud->points[index];
    if(inliers.count(index))
      cloudOutliers->points.push_back(point);
    else
      cloudInliers->points.push_back(point);
  }


  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
  return segResult;
}

//-------------------------------------------------------------------------





template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
  	//https://pcl-tutorials.readthedocs.io/en/master/voxel_grid.html
  	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);
  
  	//set region of interest
  	typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cbox;
    cbox.setMin(minPoint);
    cbox.setMax(maxPoint);
  	cbox.setInputCloud(cloud_filtered);
    cbox.filter (*cloud_region);
  
    	//crop region of interest
  	std::vector<int> roof_points_indices;
    pcl::CropBox<PointT> cbox_roof;
    cbox_roof.setMin(Eigen::Vector4f ( -1.5, -1.7, -1, 1));
    cbox_roof.setMax(Eigen::Vector4f ( 2.6, 1.7, -0.4, 1));
  	cbox_roof.setInputCloud(cloud_region);
    cbox_roof.filter (roof_points_indices);
  	//get the roof points indexes
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
    for(int point : roof_points_indices) inliers->indices.push_back(point);

    typename pcl::PointCloud<PointT>::Ptr cloud_no_roof (new pcl::PointCloud<PointT>);
    // Create the filtering object
    //https://pointclouds.org/documentation/tutorials/cluster_extraction.html#cluster-extraction
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud_region);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_no_roof);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_no_roof;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);
  // Create the filtering object
  //https://pointclouds.org/documentation/tutorials/cluster_extraction.html#cluster-extraction
  pcl::ExtractIndices<PointT> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_f);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
  
  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_p);
    
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_p, cloud_f);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
  
    pcl::SACSegmentation<PointT> seg;
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
    // TODO:: Fill in this function to find inliers for the cloud.
 	// Create the segmentation object
  
	seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
  	// Segment the largest planar component from the input cloud
 	seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if(inliers->indices.size() == 0) {
     std::cout << "Could not estimate a planar model for the given dataset." << std::endl; 
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  	//https://pointclouds.org/documentation/tutorials/cluster_extraction.html#cluster-extraction
  	// Creating the KdTree object for the search method of the extraction
  	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  	tree->setInputCloud (cloud);
  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setClusterTolerance (clusterTolerance); // 2cm
  	ec.setMinClusterSize (minSize);
  	ec.setMaxClusterSize (maxSize);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud);
  	ec.extract (cluster_indices);
  
  	int clusterId = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      
      clusters.push_back(cloud_cluster);
      
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points.";
      std::cout << " || clusterId: " << clusterId << std::endl;
      
      clusterId++;
    }
  

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}