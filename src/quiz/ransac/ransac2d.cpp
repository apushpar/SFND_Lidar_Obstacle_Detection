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
	
	std::unordered_set<int> tmpInliers;
	// TODO: Fill in this function
	int cloudSize = cloud->points.size();
	// std::cout << "cloud size: " << cloudSize << std::endl;
	// For max iterations
	int i = 0;
	while (i < maxIterations) {
	// Randomly sample subset and fit line
		tmpInliers.clear();
		// get 2 random points
		int randPointIndex1 = rand() % cloudSize;
		int randPointIndex2 = rand() % cloudSize;
		while (randPointIndex2 == randPointIndex1) {
			randPointIndex2 = rand() % cloudSize;
		}

		pcl::PointXYZ randPoint1 = cloud->points[randPointIndex1];
		pcl::PointXYZ randPoint2 = cloud->points[randPointIndex2];

		// std::cout << "point 1: " << randPoint1.x << ", " << randPoint1.y << std::endl;
		// std::cout << "point 2: " << randPoint2.x << ", " << randPoint2.y << std::endl;
		// calculate A, B, C
		int a = randPoint1.y - randPoint2.y;
		int b = randPoint2.x - randPoint1.x;
		int c = (randPoint1.x*randPoint2.y) - (randPoint2.x*randPoint1.y);
		double den = sqrt(a*a + b*b);
		// iterate over all points in the cloud
		for(int index = 0; index < cloud->points.size(); index++)
		{
			pcl::PointXYZ point = cloud->points[index];
			// measure distance
			double num = fabs(a*point.x + b*point.y + c);
			float distance = num / den;
			// std::cout << "          distance: " << distance << " num: " << num << "  den: " << den << std::endl;
			if (distance <= distanceTol) {
				tmpInliers.insert(index);
			}
		}

		if (tmpInliers.size() >  inliersResult.size()){
			inliersResult.swap(tmpInliers);
		}

		// std::cout << "Inliers size: " << inliersResult.size() << std::endl;
		i++;
	}
	

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	std::unordered_set<int> tmpInliers;
	// TODO: Fill in this function
	int cloudSize = cloud->points.size();
	// std::cout << "cloud size: " << cloudSize << std::endl;
	// For max iterations
	int i = 0;
	while (i < maxIterations) {
	// Randomly sample subset and fit line
		tmpInliers.clear();
		// get 2 random points
		int randPointIndex1 = rand() % cloudSize;
		int randPointIndex2 = rand() % cloudSize;
		int randPointIndex3 = rand() % cloudSize;
		while (randPointIndex2 == randPointIndex1) {
			randPointIndex2 = rand() % cloudSize;
		}
		while (randPointIndex1 == randPointIndex3 && randPointIndex2 == randPointIndex3) {
			randPointIndex3 = rand() % cloudSize;
		}
		pcl::PointXYZ randPoint1 = cloud->points[randPointIndex1];
		pcl::PointXYZ randPoint2 = cloud->points[randPointIndex2];
		pcl::PointXYZ randPoint3 = cloud->points[randPointIndex3];

		// std::cout << "point 1: " << randPoint1.x << ", " << randPoint1.y << std::endl;
		// std::cout << "point 2: " << randPoint2.x << ", " << randPoint2.y << std::endl;

		// calculate A, B, C, D
		int a = (randPoint2.y - randPoint1.y) * (randPoint3.z - randPoint1.z) - (randPoint2.z - randPoint1.z) * (randPoint3.y - randPoint1.y);
		int b = (randPoint2.z - randPoint1.z) * (randPoint3.x - randPoint1.x) - (randPoint2.x - randPoint1.x) * (randPoint3.z - randPoint1.z);
		int c = (randPoint2.x - randPoint1.x) * (randPoint3.y - randPoint1.y) - (randPoint2.y - randPoint1.y) * (randPoint3.x - randPoint1.x);
		int d = -1 * (a * randPoint1.x + b * randPoint1.y + c * randPoint1.z);
		double den = sqrt(a * a + b * b + c * c);
		// iterate over all points in the cloud
		for(int index = 0; index < cloud->points.size(); index++)
		{
			pcl::PointXYZ point = cloud->points[index];
			// measure distance
			double num = fabs(a * point.x + b * point.y + c * point.z + d);
			float distance = num / den;
			// std::cout << "          distance: " << distance << " num: " << num << "  den: " << den << std::endl;
			if (distance <= distanceTol) {
				tmpInliers.insert(index);
			}
		}

		if (tmpInliers.size() >  inliersResult.size()){
			inliersResult.swap(tmpInliers);
		}

		// std::cout << "Inliers size: " << inliersResult.size() << std::endl;
		i++;
	}
	

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.2);

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
