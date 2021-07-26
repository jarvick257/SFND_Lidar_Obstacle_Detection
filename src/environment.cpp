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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &f_viewer_p,
               std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> f_pp_p,
               pcl::PointCloud<pcl::PointXYZI>::Ptr f_inputCloud_p) {

  auto filteredCloud =
      f_pp_p->FilterCloud(f_inputCloud_p, 0.2, Eigen::Vector4f(-15, -5, -5, 0),
                          Eigen::Vector4f(50, 5, 5, 0));
  // renderPointCloud(f_viewer_p, filteredCloud, "inputCloud");

  // Split cloud into plane and obstacles
  auto segCloud = f_pp_p->SegmentPlane(filteredCloud, 100, 0.2);
  // renderPointCloud(f_viewer_p, segCloud.first, "obstCloud", Color(1, 0, 0));
  renderPointCloud(f_viewer_p, segCloud.second, "planeCloud",
                   Color(0.2, 0.2, 0.2));

  // Cluster obstacles
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = f_pp_p->Clustering(segCloud.first, 0.5, 10, 1000);
  int clusterId = 0;
  const std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0),
                                     Color(0, 1, 0), Color(0, 1, 1),
                                     Color(0, 0, 1), Color(1, 0, 1)};

  for (const auto &cluster : clusters) {
    std::cout << "cluster size " << f_pp_p->numPoints(cluster) << std::endl;
    renderPointCloud(f_viewer_p, cluster,
                     "obstCloud" + std::to_string(clusterId),
                     colors[clusterId % colors.size()]);
    Box box = f_pp_p->BoundingBox(cluster);
    renderBox(f_viewer_p, box, clusterId);
    ++clusterId;
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
    
    auto lidar_p = std::make_shared<Lidar>(cars, 0.0);
    auto inputCloud = lidar_p->scan();

    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    // Split cloud into plane and obstacles
    auto segCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer, segCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segCloud.second, "planeCloud", Color(0, 1, 0));

    // Cluster obstacles
    auto clusters = pointProcessor.Clustering(segCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    const std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0),
                                       Color(0, 0, 1)};

    for (const auto &cluster : clusters) {
      std::cout << "cluster size " << pointProcessor.numPoints(cluster)
                << std::endl;
      renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                       colors[clusterId % colors.size()]);
      Box box = pointProcessor.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
      ++clusterId;
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

    pcl::visualization::PCLVisualizer::Ptr viewer_p(new pcl::visualization::PCLVisualizer("3D Viewer"));

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer_p);
    // simpleHighway(viewer_p);

    auto pp_p = std::make_shared<ProcessPointClouds<pcl::PointXYZI>>();
    std::vector<boost::filesystem::path> stream =
        pp_p->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI_p(new pcl::PointCloud<pcl::PointXYZI>());

    while (!viewer_p->wasStopped()) {
      // Clear viewer
      viewer_p->removeAllPointClouds();
      viewer_p->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI_p = pp_p->loadPcd((*streamIterator).string());
      cityBlock(viewer_p, pp_p, inputCloudI_p);

      streamIterator++;
      if (streamIterator == stream.end())
        streamIterator = stream.begin();

      viewer_p->spinOnce();
    }
}
