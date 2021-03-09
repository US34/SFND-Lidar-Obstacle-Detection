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
  std::unordered_set<int> inliers;
	srand(time(NULL));
  
  while(maxIterations--){
    while(inliers.size() < 3){
      inliers.insert(rand()%(cloud->points.size()));
    }
	float x1, x2, x3, y1, y2, y3, z1, z2, z3;
    auto itr = inliers.begin();
    pcl::PointXYZ point1 = cloud->points[*itr];
    x1 = point1.x;
    y1 = point1.y;
    z1 = point1.z;
      
    itr++;
    pcl::PointXYZ point2 = cloud->points[*itr];
    x2 = point2.x;
    y2 = point2.y;
    z2 = point2.z;
      
    itr++;
    pcl::PointXYZ point3 = cloud->points[*itr];
    x3 = point3.x;
    y3 = point3.y;
    z3 = point3.z;
    

	auto A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
	auto B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
	auto C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
	auto D = -A*x1 - B*y1 - C*z1;
     
    for(int i=0; i<cloud->points.size(); i++){
      if(inliers.count(i) > 0){
         continue;
      }
      pcl::PointXYZ point = cloud->points[i];
      float x = point.x;
      float y = point.y;
      float z = point.z;
      
      
			auto d = fabs(A*x+B*y+C*z+D) / sqrt(A*A+B*B+C*C); 
      if( d < distanceTol){
        inliers.insert(i);  
      }
		
		}
    if(inliersResult.size() < inliers.size()){
       inliersResult = inliers;
    }
    inliers.clear();
       
    }
	
	// TODO: Fill in this function
	
	// For max iterations 
	/*for( int i=1, i <= maxIterations-1, i++){
      pcl::PointXYZ point2 = cloud->points[i];
      pcl::PointXYZ point1 = cloud->points[i-1];
      auto y1 = point1.y;
      auto y2 = point2.y;
      auto x1 = point1.x;
      auto x2 = point2.x;
      
	// Randomly sample subset and fit line
	  auto A = y1-y2 ;
      auto B = x2-x1;
   	  int indice = 0;
      inliers.clear();
	// Measure distance between every point and fitted line
      for( pcl::PointXYZ j : cloud->points){
        auto d = abs(A*j.x + B*j.y - A*x1 -B*y1) / sqrt(A**2 + B**2) ;
        
	// If distance is smaller than threshold count it as inlier
        if( d < distanceTol ){
          inliers.insert(indice)
        }
        
        int indice++;
      }
      if (inliersResult.count() < inliers.count()){
        inliers = inliersResult;
      }*/
    
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
