/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{/*
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    Lidar* lidar = new Lidar(cars,0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

    //renderRays(viewer,lidar->position,pointCloud);  
    //renderPointCloud(viewer,inputCloud,"inputCloud");  

    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedCloud = pointProcessor->RansacPlane(inputCloud,30,0.3);
    renderPointCloud(viewer,segmentedCloud.first,"ObstaclesCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentedCloud.second,"PlaneCloud",Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor->CustomClustering(segmentedCloud.first,3.5f,2,50);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster:clusters)
    {
        std::cout << "Cluster size ";
        pointProcessor->numPoints(cluster);
            
        //renderPointCloud(viewer,cluster,"obstCloud" + std::to_string(clusterId),colors[clusterId]);
        Box box=pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    */
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud,0.1,Eigen::Vector4f(-15,-5,-2,1),Eigen::Vector4f(15,8,5,1));    
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->RansacPlane(filterCloud,25,0.15);
    
    renderPointCloud(viewer,segmentedCloud.first,"ObstaclesCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentedCloud.second,"PlaneCloud",Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->CustomClustering(segmentedCloud.first,0.6f,25,1000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster:clusters)
    {
        //std::cout << "Cluster size ";
        pointProcessorI->numPoints(cluster);
            
        //renderPointCloud(viewer,cluster,"obstCloud" + std::to_string(clusterId),colors[clusterId]);
        Box box=pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    //renderPointCloud(viewer,inputCloud,"inputCloud");
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
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}