// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    std::cout << "cloud_filtered has " << cloud_filtered->points.size () << " data points" << std::endl;

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered_cropped (new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> cropBox(true);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(cloud_filtered);
    cropBox.filter(*cloud_filtered_cropped);

    std::cout << "cloud_filtered_cropped has " << cloud_filtered_cropped->points.size () << " data points" << std::endl;

    // remove points hitting on the car's roof top
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_filtered_cropped);
    std::vector<int> indices;
    roof.filter(indices);

    std::cout << "Roof indices has found " << indices.size () << " data points" << std::endl;

    pcl::PointIndices::Ptr inliers { new pcl::PointIndices};
    for(int point : indices )
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_filtered_cropped);
    extract.setIndices (inliers);
    extract.setNegative(true);
    extract.filter (*cloud_filtered_cropped);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered_cropped;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for( int index : inliers->indices)
    {
        planeCloud->points.push_back( cloud->points[index] );
    }

    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
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

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> PclPointProcessor<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud (cloud);
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = this->SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> PclPointProcessor<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for( int i : it->indices )
            cloud_cluster->points.push_back (cloud->points[i]);

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);

        std::cout << "PointCloud representing the cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
StudentPointProcessor<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    std::unordered_set<int> inliers_indices = RansacPlane(cloud, maxIterations, distanceThreshold);
    if (inliers_indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateCloudsRamon(inliers_indices,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return segResult;
}

template<typename PointT>
std::unordered_set<int> StudentPointProcessor<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	while( maxIterations--)
	{

		std::unordered_set<int> temp_inliers;

		while( temp_inliers.size() < 3)
			temp_inliers.insert( rand()%cloud->size() );

		auto it = temp_inliers.begin();
		const PointT & p1 = cloud->points[*it++];
		const PointT & p2 = cloud->points[*it++];
		const PointT & p3 = cloud->points[*it];

		double i = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		double j = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x-p1.x)*(p3.z-p1.z);
		double k = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);


		double A = i;
		double B = j;
		double C = k;
		double D = -(i*p1.x + j*p1.y + k*p1.z);


		for(int p = 0; p < cloud->points.size(); p++)
		{
			if( temp_inliers.count(p) ) continue;

			const PointT & point = cloud->points[p];
			double distance = fabs(A*point.x + B*point.y + C*point.z + D)/sqrt( A*A + B*B + C*C);
			if( distance < distanceTol)
				temp_inliers.insert(p);
		}

		if( temp_inliers.size() > inliersResult.size() )
		{
			inliersResult.clear();
			inliersResult = temp_inliers;
		}

	}

	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
StudentPointProcessor<PointT>::SeparateCloudsRamon(const std::unordered_set<int> & inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for( int index : inliers)
    {
        planeCloud->points.push_back( cloud->points[index] );
    }

    for( int index = 0; index < cloud->points.size(); index++)
    {
        if( inliers.count(index) == 0)
            obstCloud->points.push_back( cloud->points[index] );
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(obstCloud, planeCloud);
}


template<typename PointT>
void StudentPointProcessor<PointT>::Proximity(const std::vector<std::vector<float>>& points, KdTree * tree, float distanceTol, int pointIndex, std::vector<bool> & points_processed, std::vector<int> & cluster )
{
	if( points_processed[pointIndex] ) return;

	points_processed[pointIndex] = true;

	cluster.push_back(pointIndex);

	std::vector<int> nearby = tree->search(points[pointIndex], distanceTol);
	for( int i : nearby )
	{
		Proximity(points, tree, distanceTol, i, points_processed, cluster);
	}
}

template<typename PointT>
std::vector<std::vector<int>> StudentPointProcessor<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
 
	std::vector<bool> points_processed( points.size(), false);

	for( int i = 0; i < points.size(); i++)
	{
		if( points_processed[i] == false )
		{
			std::vector<int> new_cluster;
			Proximity( points, tree, distanceTol, i, points_processed, new_cluster);
			clusters.push_back(new_cluster);
		}
	}

	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> StudentPointProcessor<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree tree;
    std::vector<std::vector<float>> cloud_points;
    // TODO: Should order before inserting in the tree in order to have a more balanced tree
    // TODO: Make euclieanCluster be able to accept pcl::PointCloud<PointT>::Ptr cloud as input
    for( int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> point(cloud->points[i].data, cloud->points[i].data+3);   
        tree.insert(point, i);
        cloud_points.push_back(point);
    }

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud_points, &tree, clusterTolerance);
    for (auto & cluster : cluster_indices)
    {
        // discard cluster if it is not big enough
        if ( cluster.size() < minSize) continue;

        // TODO: what should we do when the cluster is too big?
        if ( cluster.size() > maxSize )
        {
            std::cout << "WARNING: Cluster size too big: " << cluster.size() << std::endl;
            continue;
        }

        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for( int i : cluster )
            cloud_cluster->points.push_back (cloud->points[i]);

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
