#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include <iostream>

struct PipelineParams
{
    enum ProcessorType { PCL, STUDENT };

    // General parameters
    ProcessorType processorType;
    uint32_t streamInterval_ms;
    std::string dataSet;

    // Filtering params
    float filterResolution;
    Eigen::Vector4f cropMinPoint;
    Eigen::Vector4f cropMaxPoint;

    // SegmentPlane params
    int spMaxIterations;
    float spDistanceThreshold;

    // Clustering params
    float clusterTolerance;
    int clusterMinSize;
    int clusterMaxSize;

    // Visualization params
    bool renderInputCloud;
    bool renderFilteredCloud;
    bool renderObstacles;
    bool renderPlane;
    bool renderClusters;
    bool renderBoxes;

    // Extra challenge, cycles follower
    bool followCyclist;
    double boxTolerance;

    bool fromFile(const std::string & filePath)
    {
        ifstream file_(filePath);

        if( !file_.is_open() )
        {
            std::cerr << "Params file not found!" << std::endl;
            return false;
        }

        std::string line_;
        int i = 0;
        while( getline(file_, line_ ) )
        {
            if( line_[0] == '#') continue;
            if( line_.empty() ) continue;

            std::stringstream check1(line_);
            std::string paramName;

            check1 >> paramName;
            if( paramName == "filterResolution:") { check1 >> filterResolution; }
            else if( paramName == "processorType:")
            {
                std::string type_str;
                check1 >> type_str;
                if( type_str == "PCL" ) processorType = PCL;
                else if( type_str == "STUDENT" ) processorType = STUDENT;
                else
                {
                    std::cerr << "Unrecognized processor type: " << type_str << std::endl;
                    processorType = PCL;
                }
            }
            else if( paramName == "streamInterval_ms:") { check1 >> streamInterval_ms; }
            else if( paramName == "dataSet:") { check1 >> dataSet; }
            else if( paramName == "cropMinPoint:")
            {
                double x,y,z,i;
                check1 >> x >> y >> z >> i;
                cropMinPoint = Eigen::Vector4f (x, y, z, i);
            }
            else if( paramName == "cropMaxPoint:") 
            {
                double x,y,z,i;
                check1 >> x >> y >> z >> i;
                cropMaxPoint = Eigen::Vector4f (x, y, z, i);
            }
            else if( paramName == "spMaxIterations:") { check1 >> spMaxIterations; }
            else if( paramName == "spDistanceThreshold:") { check1 >> spDistanceThreshold; }
            else if( paramName == "clusterTolerance:") { check1 >> clusterTolerance; }
            else if( paramName == "clusterMinSize:") { check1 >> clusterMinSize; }
            else if( paramName == "clusterMaxSize:") { check1 >> clusterMaxSize; }
            else if( paramName == "renderInputCloud:") { check1 >> renderInputCloud; }
            else if( paramName == "renderFilteredCloud:") { check1 >> renderFilteredCloud; }
            else if( paramName == "renderObstacles:") { check1 >> renderObstacles; }
            else if( paramName == "renderPlane:") { check1 >> renderPlane; }
            else if( paramName == "renderClusters:") { check1 >> renderClusters; }
            else if( paramName == "renderBoxes:") { check1 >> renderBoxes; }
            else if( paramName == "followCyclist:") { check1 >> followCyclist; }
            else if( paramName == "boxTolerance:") { check1 >> boxTolerance; }
            else
            {
                std::cerr << "Unrecognized pipeline parameter: " << paramName << std::endl;
                assert(0);
            }
        }

        file_.close();

        return true;
    }

    void print()
    {
        std::cout << "Filtering: filterRes: " << filterResolution << std::endl;
        std::cout << "Filtering: cropMinPoint: " << cropMinPoint << std::endl;
        std::cout << "Filtering: cropMaxPoint: " << cropMaxPoint << std::endl;
        std::cout << "SegmentPlane: spMaxIterations: " << spMaxIterations << std::endl;
        std::cout << "SegmentPlane: spDistanceThreshold: " << spDistanceThreshold << std::endl;    
        std::cout << "Clustering: clusterTolerance: " << clusterTolerance << std::endl;    
        std::cout << "Clustering: clusterMinSize: " << clusterMinSize << std::endl;    
        std::cout << "Clustering: clusterMaxSize: " << clusterMaxSize << std::endl;    
    }
};

#endif
