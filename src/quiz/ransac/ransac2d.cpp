/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <math.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
  for(int interation = 0; interation < maxIterations; interation++) { 
    // Randomly sample subset and fit line
    int cloudDataLen = cloud->width;
    int randomID1, randomID2;
    randomID1 = randomID2 = rand() % cloudDataLen;
    while(randomID1 == randomID2) randomID2 = rand() % cloudDataLen;
    pcl::PointXYZ pt1 = cloud->points[randomID1];
    pcl::PointXYZ pt2 = cloud->points[randomID2];
	// Measure distance between every point and fitted line
    float A = pt1.y - pt2.y;
    float B = pt2.x - pt1.x;
    float C = pt1.x*pt2.y - pt2.x*pt1.y;
	// If distance is smaller than threshold count it as inlier
    std::unordered_set<int> tempInliersResult;
    for(int i = 0; i < cloudDataLen; i++) {
      pcl::PointXYZ pt = cloud->points[i];
      float d = std::fabs(A*pt.x + B*pt.y + C)/sqrt(A*A + B*B);
      if(d <= distanceTol) tempInliersResult.insert(i);
    }
  
	// Return indicies of inliers from fitted line with most inliers
	if(tempInliersResult.size() > inliersResult.size()) inliersResult = tempInliersResult;
  }
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
    while(randomID1 == randomID2 == randomID3) {
      randomID2 = rand() % cloudDataLen;
      randomID3 = rand() % cloudDataLen;
    }
    pcl::PointXYZ pt1 = cloud->points[randomID1];
    pcl::PointXYZ pt2 = cloud->points[randomID2];
    pcl::PointXYZ pt3 = cloud->points[randomID3];
	// Measure distance between every point and fitted line
    double A = (pt2.y - pt1.y)*(pt3.z - pt1.z) - (pt2.z - pt1.z)*(pt3.y - pt1.y);
    double B = (pt2.z - pt1.z)*(pt3.x - pt1.x) - (pt2.x - pt1.x)*(pt3.z - pt1.z);
    double C = (pt2.x - pt1.x)*(pt3.y - pt1.y) - (pt2.y - pt1.y)*(pt3.x - pt1.x);
    double D = -(A*pt1.x + B*pt1.y + C*pt1.z);
	// If distance is smaller than threshold count it as inlier
    std::unordered_set<int> tempInliersResult;
    for(int i = 0; i < cloudDataLen; i++) {
      pcl::PointXYZ pt = cloud->points[i];
      double d = std::fabs(A*pt.x + B*pt.y + C*pt.z + D)/sqrt(A*A + B*B + C*C);
      if(d <= distanceTol) tempInliersResult.insert(i);
    }
  
	// Return indicies of inliers from fitted line with most inliers
	if(tempInliersResult.size() > inliersResult.size()) inliersResult = tempInliersResult;
  }
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 2000, 0.5);
  	std::cout<< "Cloud total amount: " << cloud->points.size() << std::endl;
  	std::cout<< "inliers total amount: " << inliers.size() << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
