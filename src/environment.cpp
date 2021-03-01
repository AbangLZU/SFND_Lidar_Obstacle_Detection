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

pcl::PointCloud<pcl::PointXYZI>::Ptr loadCloud(std::string pcd_path){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_path, *cloud);
    return cloud;
}

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr & viewer){
void cityBlock(pcl::visualization::PCLVisualizer::Ptr & viewer, ProcessPointClouds<pcl::PointXYZI> processor,
               pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_data){
    // ----------------------------------------------------
    // -----       play with real pcd data            -----
    // ----------------------------------------------------
//    ProcessPointClouds<pcl::PointXYZI> processor;
//    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_data = loadCloud("../src/sensors/data/pcd/data_1/0000000000.pcd");
//    renderPointCloud(viewer, pcd_data, "pcd_data");

    Eigen::Vector4f min_point(-20.0, -10.0, -20.0, 1);
    Eigen::Vector4f max_point(20, 10, 20, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = processor.FilterCloud(pcd_data,
                                                                                0.3, min_point, max_point);
//    renderPointCloud(viewer, filtered_cloud, "filtered_cloud");
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr , pcl::PointCloud<pcl::PointXYZI>::Ptr> seg_pair =
            processor.SegmentPlane(filtered_cloud, 40, 0.3);

    renderPointCloud(viewer, seg_pair.first, "plane_cloud", Color(1, 1, 1));
//    renderPointCloud(viewer, seg_pair.second, "obstacle_cloud", Color(1, 0, 0));

    auto obstacle_cloud = seg_pair.second;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters =
            processor.Clustering(obstacle_cloud, 0.4, 20, 200);

    for (int i = 0; i < clusters.size(); ++i) {
        renderPointCloud(viewer, clusters[i], "cluster "+std::to_string(i), Color(0, 1, 0));
        Box boxes = processor.BoundingBox(clusters[i]);
        renderBox(viewer, boxes, i, Color(1, 0, 0));
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
    
    // Create lidar sensor

    Lidar *mLidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr = mLidar->scan();

//    renderRays(viewer, mLidar->position, pc_ptr);
//    renderPointCloud(viewer, pc_ptr, "input_cloud");
    
    // Create point processor

    ProcessPointClouds<pcl::PointXYZ> point_processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr , pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_pair =
            point_processor.SegmentPlane(pc_ptr, 100, 0.2);

    renderPointCloud(viewer, pc_pair.first, "plane_cloud", Color(0, 1, 0));
    renderPointCloud(viewer, pc_pair.second, "obstacle_cloud", Color(1, 0, 0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud = pc_pair.second;
    auto cluster_list = point_processor.Clustering(obstacle_cloud, 1.0, 3, 100);

    std::vector<Color> color_list = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};


    for (int i = 0; i < cluster_list.size(); ++i) {
        renderPointCloud(viewer, cluster_list[i], "cluster "+std::to_string(i), color_list[i%color_list.size()]);
        Box boxes = point_processor.BoundingBox(cluster_list[i]);
        renderBox(viewer, boxes, i, color_list[i%color_list.size()]);
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
//    simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> cloud_processer;

    std::vector<boost::filesystem::path> path_list = cloud_processer.streamPcd("../src/sensors/data/pcd/data_1");
    auto pcd_iter = path_list.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        current_cloud = loadCloud(pcd_iter->string());
        cityBlock(viewer, cloud_processer, current_cloud);

        pcd_iter++;
        if (pcd_iter == path_list.end()){
            pcd_iter = path_list.begin();
        }

        viewer->spinOnce ();
    } 
}