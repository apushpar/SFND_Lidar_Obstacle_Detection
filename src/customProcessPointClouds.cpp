// PCL lib Functions for processing point clouds 

#include "customProcessPointClouds.h"
#include <unordered_set>
#include "quiz/cluster/kdtree.h"
#include <unordered_set>

//constructor:
template<typename PointT>
CustomProcessPointClouds<PointT>::CustomProcessPointClouds() {}


//de-constructor:
template<typename PointT>
CustomProcessPointClouds<PointT>::~CustomProcessPointClouds() {}


template<typename PointT>
void CustomProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr CustomProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>), cloud_p (new pcl::PointCloud<PointT>);
    typename pcl::VoxelGrid<PointT> sor;
    typename pcl::CropBox<PointT> cb;
    // voxel grid
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_f);

    // cropbox to limit region of interest
    cb.setInputCloud(cloud_f);
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.setNegative(false);
    cb.filter(*cloud_f);
    
    // cropbox to remove the points on the roof of the car
    cb.setInputCloud(cloud_f);
    // cb.setMin(Eigen::Vector4f (-1.5, -2, -5, 1)); // akshay soln
    cb.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1)); // soln from udacity
    // cb.setMax(Eigen::Vector4f (3, 2, 10, 1)); // akshay soln
    cb.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1)); //soln from udacity
    cb.setNegative(true);
    cb.filter(*cloud_f);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_f;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CustomProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacle (new pcl::PointCloud<PointT>), cloud_other (new pcl::PointCloud<PointT>);

    for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloud_obstacle->points.push_back(point);
		else
			cloud_other->points.push_back(point);
	}
    

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obstacle, cloud_other);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CustomProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    std::unordered_set<int> inliersResult;
	
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
		PointT randPoint1 = cloud->points[randPointIndex1];
		PointT randPoint2 = cloud->points[randPointIndex2];
		PointT randPoint3 = cloud->points[randPointIndex3];

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
			PointT point = cloud->points[index];
			// measure distance
			double num = fabs(a * point.x + b * point.y + c * point.z + d);
			float distance = num / den;
			// std::cout << "          distance: " << distance << " num: " << num << "  den: " << den << std::endl;
			if (distance <= distanceThreshold) {
				tmpInliers.insert(index);
			}
		}

		if (tmpInliers.size() >  inliersResult.size()){
			inliersResult.swap(tmpInliers);
		}

		// std::cout << "Inliers size: " << inliersResult.size() << std::endl;
		i++;
	}
	
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    return segResult;
}

template<typename PointT>
void CustomProcessPointClouds<PointT>::Proximity(typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr clusterCloud, std::unordered_set<int> &processed_ids, int id, KdTree* tree, float distanceTol)
{
	processed_ids.insert(id);
    clusterCloud->points.push_back(cloud->points[id]);
	std::vector<int> nearby = tree->search({cloud->points[id].x, cloud->points[id].y}, distanceTol);
	for(int index : nearby)
	{
		if (!processed_ids.count(index))
			Proximity(cloud, clusterCloud, processed_ids, index, tree, distanceTol);
	}
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> CustomProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersResult;

    KdTree* tree = new KdTree;
  
    for (int i=0; i<cloud->points.size(); i++) 
    	tree->insert({cloud->points[i].x, cloud->points[i].y},i);
    
    std::vector<std::vector<int>> clusters;
	std::unordered_set<int> processed_ids;
	for (int i=0; i<cloud->points.size(); i++)
	{
		if (!processed_ids.count(i))
		{
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
			Proximity(cloud, clusterCloud, processed_ids, i, tree, clusterTolerance);
            if (clusterCloud->points.size() >= minSize && clusterCloud->points.size() <= maxSize)
                clustersResult.push_back(clusterCloud);
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clustersResult.size() << " clusters" << std::endl;

    return clustersResult;
}


template<typename PointT>
Box CustomProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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
void CustomProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr CustomProcessPointClouds<PointT>::loadPcd(std::string file)
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
std::vector<boost::filesystem::path> CustomProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}