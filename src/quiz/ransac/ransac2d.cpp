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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
    while (maxIterations--) {
      std::unordered_set<int> inliers;

      // Randomly sample subset and fit line
      while (inliers.size() < 2) {
        inliers.insert(rand() % cloud->points.size());
      }

      auto it = inliers.begin();
      const float x1 = cloud->points[*it].x;
      const float y1 = cloud->points[*it].y;
      it++;
      const float x2 = cloud->points[*it].x;
      const float y2 = cloud->points[*it].y;

      const float a = y2 - y1;
      const float b = x2 - x1;
      const float c = (x1 * y2 - x2 * y1);

      for (int index = 0; index < cloud->points.size(); index++) {
        if (inliers.count(index) > 0) {
          continue;
        }

        const pcl::PointXYZ point = cloud->points[index];
        const float x3 = point.x;
        const float y3 = point.y;

        const float d = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b);
        if (d <= distanceTol) {
          inliers.insert(index);
        }
      }

      if (inliers.size() > inliersResult.size()) {
        inliersResult = inliers;
      }

      // Measure distance between every point and fitted line
      // If distance is smaller than threshold count it as inlier
    }

    // Return indicies of inliers from fitted line with most inliers

    return inliersResult;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "Usage: ./quizRansac <iterations> <maxDistance>" << std::endl;
    return 1;
  }

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

  const int numIterations = std::atoi(argv[1]);
  const float maxDistance = std::atof(argv[2]);
  std::cout << "Num Iterations: " << numIterations << std::endl;
  std::cout << "Max Distance:   " << maxDistance << std::endl;
  std::unordered_set<int> inliers = Ransac(cloud, numIterations, maxDistance);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
