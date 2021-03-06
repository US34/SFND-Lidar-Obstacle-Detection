// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"




//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    KdTree* tree = new KdTree;
  
	  for (int i=0; i< cloud->points.size(); i++){ 
      tree->insert(cloud->points[i],i);
  	}
    std::cout << "tree declared" << std::endl;
    std::vector<std::vector<int>> clusters_id = Cluster(cloud, tree, clusterTolerance);
  	
  	for(std::vector<int> cluster : clusters_id)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		if (cluster.size()>=minSize && cluster.size()<=maxSize) {
      		for(int indice: cluster){
  	    		clusterCloud->points.push_back(pcl::PointXYZI(cloud->points[indice]));
    		  }
      }
  	
      clusterCloud->width = clusterCloud->points.size();
	    clusterCloud->height = 1;

      clusters.push_back(clusterCloud);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "KDTree clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::Cluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol)
{


	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(cloud->points.size(), false);

	int i = 0;
	while(i < cloud->points.size()){
		if(processed[i])
		{
			i++;
			continue;
		} 
    //std::cout << "clustering" << std::endl;
		std::vector<int> cluster;
		clusterHelper(i, cloud, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}
		
	std::cout << "number of cluster" << clusters.size() << std::endl;
	return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster1, std::vector<bool> processed, KdTree* tree, float distanceTol)
{
	processed[indice] = true;
	cluster1.push_back(indice);
	//std::cout << "cluster size" << cluster1.size() << std::endl;
	std::vector<int> nearest = tree->search(cloud->points[indice], distanceTol);
  //std::cout << "searching" << nearest.size() << std::endl;
	for(int id : nearest)
	{
		if(!processed[id]){
			clusterHelper(id, cloud, cluster1, processed, tree, distanceTol);
		}
	}
}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
    PointT point1 = cloud->points[*itr];
    x1 = point1.x;
    y1 = point1.y;
    z1 = point1.z;
      
    itr++;
    PointT point2 = cloud->points[*itr];
    x2 = point2.x;
    y2 = point2.y;
    z2 = point2.z;
      
    itr++;
    PointT point3 = cloud->points[*itr];
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
      PointT point = cloud->points[i];
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
  return inliersResult;

}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  	pcl::VoxelGrid<PointT> sor;
  	typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>); 
  	sor.setInputCloud(cloud);
  	sor.setLeafSize (filterRes, filterRes, filterRes);
  	sor.filter (*cloudFiltered);

  	typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>); 
  	pcl::CropBox<PointT> region(true);
  	region.setMin(minPoint);
  	region.setMax(maxPoint);
  	region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
   
  	std::vector<int> indices;
  	pcl::CropBox<PointT> roof(true);
  	region.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  	region.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  	region.setInputCloud(cloudRegion);
    region.filter(indices);
  
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	for(int point : indices){
      inliers->indices.push_back(point);
    }
  
  	pcl::ExtractIndices<PointT> extract;
  	extract.setInputCloud(cloudRegion);
  	extract.setIndices(inliers);
  	extract.setNegative(true);
  	extract.filter(*cloudRegion);
  	
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>());
  	typename pcl::PointCloud<PointT>::Ptr obstacle (new pcl::PointCloud<PointT>());
  	for(int index : inliers->indices){
      plane->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacle);
         
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // TODO:: Fill in this function to find inliers for the cloud.
  	pcl::SACSegmentation<PointT> seg;
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
  	seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
  	if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  	tree->setInputCloud (cloud);
  	std::vector<pcl::PointIndices> cluster_indices;
  	typename pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setClusterTolerance (clusterTolerance); // 2cm
  	ec.setMinClusterSize (minSize);
  	ec.setMaxClusterSize (maxSize);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud);
  	ec.extract (cluster_indices);
    
    for(pcl::PointIndices cluster : cluster_indices){
      typename pcl::PointCloud<PointT>::Ptr ptr_clust(new pcl::PointCloud<PointT>);
      for(int index : cluster.indices){
        ptr_clust->points.push_back(cloud->points[index]);
      } 
      
      ptr_clust->width = ptr_clust->points.size();
      ptr_clust->height = 1;
      ptr_clust->is_dense = true;
      
      clusters.push_back(ptr_clust);
    }
  	
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
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
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}