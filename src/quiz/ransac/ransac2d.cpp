/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
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

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	srand(time(NULL));
	
	const size_t n = cloud->points.size();
	std::vector<int> tmp;
    tmp.reserve(n);
    std::vector<int> inliersResult;
    inliersResult.reserve(n);

	// TODO: Fill in this function
	// For max iterations 
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	for(int i = 0 ; i < maxIterations; ++i) {
		// select two points
		int p1 = std::rand() % n;
		int p2 = p1, p3 = p1;
		while (p1 == p2) {
			p2 = std::rand() % n;
		}

		while (p3 == p1 || p3 == p2) {
			p3 = std::rand() % n;
		}

		const double x1 = cloud->points[p1].x, x2 = cloud->points[p2].x, x3 = cloud->points[p3].x;
		const double y1 = cloud->points[p1].y, y2 = cloud->points[p2].y, y3 = cloud->points[p3].y;
		const double z1 = cloud->points[p1].z, z2 = cloud->points[p2].z, z3 = cloud->points[p3].z;

		const double a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		const double b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		const double c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		const double d = -(a*x1 + b*y1 + c*z1);

		const double norm = std::sqrt(a*a + b*b + c*c);
		tmp.clear();
		for (size_t j = 0; j < cloud->points.size(); ++j) {
			const auto& point = cloud->points[j];
			const double dist = std::abs(a*point.x + b*point.y + c*point.z + d) / norm;
        
			if (dist <= distanceTol) {
				tmp.push_back(j);
			}
		}

		if (tmp.size() > inliersResult.size()) {
			swap(inliersResult, tmp);
		}
	}
	
	return {inliersResult.begin(), inliersResult.end()};
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	std::unordered_set<int> tmp;
	const size_t n = cloud->points.size();
	// TODO: Fill in this function
	// For max iterations 
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	for(int i = 0 ; i < maxIterations; ++i) {
		// select two points
		int p1 = std::rand() % n;
		int p2 = p1;
		while (p1 == p2) {
			p2 = std::rand() % n;
		}

		const double x1 = cloud->points[p1].x, x2 = cloud->points[p2].x;
		const double y1 = cloud->points[p1].y, y2 = cloud->points[p2].y;
		
		const double a = y1 - y2;
		const double b = x2 - x1;
		const double c = x1*y2 - x2*y1;

		const double norm = std::sqrt(a*a + b*b);
		tmp.clear();
		for (size_t j = 0; j < cloud->points.size(); ++j) {
			const double dist = std::abs(a*cloud->points[j].x + b*cloud->points[j].y + c) / norm;
			if (dist <= distanceTol) {
				tmp.insert(j);
			}
		}

		if (tmp.size() > inliersResult.size()) {
			inliersResult = tmp;
		}
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
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

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
