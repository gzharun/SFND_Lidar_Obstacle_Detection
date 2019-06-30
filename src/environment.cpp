/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

namespace {
    template<typename PointT>
    float iouXY(typename pcl::PointCloud<PointT>::Ptr a, typename pcl::PointCloud<PointT>::Ptr b) {
        PointT minPoint1, maxPoint1;
        pcl::getMinMax3D(*a, minPoint1, maxPoint1);
        PointT minPoint2, maxPoint2;
        pcl::getMinMax3D(*b, minPoint2, maxPoint2);

        float iou = 0.0f;
        float minX = std::max(minPoint1.x, minPoint2.x);
        float minY = std::max(minPoint1.y, minPoint2.y);
        float maxX = std::min(maxPoint1.x, maxPoint2.x);
        float maxY = std::min(maxPoint1.y, maxPoint2.y);

        if (minX >= maxX || minY >= maxY) {
            return 0.0f;
        }

        float inter = (maxX - minX) * (maxY - minY);
        float un = (maxPoint1.x - minPoint1.x) * (maxPoint1.y - minPoint1.y) +
                   (maxPoint2.x - minPoint2.x) * (maxPoint2.y - minPoint2.y) -
                   inter;

        return inter / un;        
    }
}

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    constexpr float filterRes = 0.2f;
    const Eigen::Vector4f minPoint = {-10.0f, -6.0f, -2.0f, 1};
    const Eigen::Vector4f maxPoint = {40.0f, 8.0f, 2.0f, 1};
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI.FilterCloud(inputCloud, filterRes, minPoint, maxPoint);

    auto clouds = pointProcessorI.SegmentPlane(filterCloud, 100, 0.25);
    //renderPointCloud(viewer, clouds.first, "ObstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, clouds.second, "RoadCloud", Color(1, 1, 1));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(clouds.first, 0.5, 10, 1000);

    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), Color(1,0,0));
        ++clusterId;
      
        //Box box = pointProcessorI.BoundingBox(cluster);
        BoxQ box = pointProcessorI.BoundingBoxQ(cluster);
        renderBox(viewer, box, clusterId);
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    auto lidar = new Lidar(cars, 0.0);
    //renderRays(viewer, lidar->position, lidar->scan());
    //renderPointCloud(viewer, lidar->scan(), "PCD");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> proc{};
    auto clouds = proc.SegmentPlane(lidar->scan(), 100, 0.2);
    renderPointCloud(viewer, clouds.first, "ObstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, clouds.second, "RoadCloud");

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = proc.Clustering(clouds.first, 2.0, 3, 100);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        proc.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
        ++clusterId;

        pcl::PointXYZ origMinPoint, origMaxPoint;
        // Calculate original bbox params, keep z coordinate
        pcl::getMinMax3D(*cluster, origMinPoint, origMaxPoint);
      
        //Box box = proc.BoundingBox(cluster);
        BoxQ box = proc.BoundingBoxQ(cluster);
        renderBox(viewer,box,clusterId);
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}