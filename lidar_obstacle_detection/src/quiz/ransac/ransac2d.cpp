/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <cmath>
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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
std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations, randomly sample subset and fit line
	while(maxIterations--)
	{	// For each iteration, randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size() < 2)
			inliers.insert(rand() % cloud->points.size());

		float x1, x2, y1, y2, z1, z2;
		// Get two points from the inliers
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		// Get the next point from the inliers and avoid iterating over the same point
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		float a = y2 - y1; // slope of the line
		float b = x1 - x2; // y-intercept of the line
		float c = x2 * y1 - x1 * y2; // x-intercept of the line

		// Measure distance between every point and fitted line
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// If the point is already an inlier, skip it
			if(inliers.count(index))
				continue;

			// Measure distance between every point and fitted line
			pcl::PointXYZ& point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;
			float z3 = point.z;
			float distance = fabs(a*x3 + b*y3 + c) / sqrt(a*a + b*b);

			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol)
				inliers.insert(index);
		}

		// If inliers is larger than inliersResult, update inliersResult
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	// Return the inliers from the fitted line with most inliers
	return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations, randomly sample subset and fit line
	while(maxIterations--)
	{	// For each iteration, randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size() < 2)
			inliers.insert(rand() % cloud->points.size());

		float x1, x2, y1, y2;
		// Get two points from the inliers
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		// Get the next point from the inliers and avoid iterating over the same point
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a = y2 - y1; // slope of the line
		float b = x1 - x2; // y-intercept of the line
		float c = x2 * y1 - x1 * y2; // x-intercept of the line

		// Measure distance between every point and fitted line
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// If the point is already an inlier, skip it
			if(inliers.count(index))
				continue;

			// Measure distance between every point and fitted line
			pcl::PointXYZ& point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;
			float distance = fabs(a*x3 + b*y3 + c) / sqrt(a*a + b*b);

			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol)
				inliers.insert(index);
		}

		// If inliers is larger than inliersResult, update inliersResult
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	// Return the inliers from the fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// Create 3D data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// Run RANSAC algorithm
	std::unordered_set<int> inliers = Ransac(cloud, 1000, 1.0);

	// Run RANSAC algorithm on 3D data
	//std::unordered_set<int> inliers = Ransac3D(cloud, 1000, 1.0);

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
