// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <unordered_set>
#include <ctime>
#include <chrono>
#include "render/box.h"

#include "kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds() {}
    //deconstructor
    virtual ~ProcessPointClouds() {}

    // The following functions have to be implemented using derived classes
    virtual std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) = 0;

    virtual std::vector<typename pcl::PointCloud<PointT>::Ptr>
    Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) = 0;

    // Common filtering function
    typename pcl::PointCloud<PointT>::Ptr
    FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    // Basic functions common to both implementations
    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};

// Implementation based on PCL used during the course
template<typename PointT>
class PclPointProcessor : public ProcessPointClouds<PointT> {
public:

    //constructor
    PclPointProcessor() {}
    //deconstructor
    virtual ~PclPointProcessor() {}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
};

// Implementation by student
template<typename PointT>
class StudentPointProcessor : public ProcessPointClouds<PointT> {
public:

    // Simple helper class to create a plane given 3 points and calculate distance to any given point
    struct PlaneParams
    {
        PlaneParams() : A(0.0), B(0.0), C(0.0), D(0.0), numInliers(0) {}

        PlaneParams(const PointT & p1, const PointT & p2, const PointT & p3) : numInliers(0)
        {
    		A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
            B = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x-p1.x)*(p3.z-p1.z);
            C = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);
            D = -(A*p1.x + B*p1.y + C*p1.z);
        }

        PlaneParams& operator=(const PlaneParams& other)
        {
            A = other.A;
            B = other.B;
            C = other.C;
            D = other.D;
            numInliers = other.numInliers;

            return *this;
        }

        double distanceToPoint(const PointT & point)
        {
            return fabs(A*point.x + B*point.y + C*point.z + D)/sqrt( A*A + B*B + C*C);
        }

   		double A, B, C, D;
        uint32_t numInliers;
    };

    //constructor
    StudentPointProcessor() {}
    //deconstructor
    virtual ~StudentPointProcessor() {}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);


    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

protected:

    void Proximity(typename pcl::PointCloud<PointT>::Ptr points, KdTree<PointT> * tree, float distanceTol, int pointIndex, std::vector<bool> & points_processed, std::vector<int> & cluster );
};


#endif /* PROCESSPOINTCLOUDS_H_ */