/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "params.h"

static std::string paramsFilePath;

static const PipelineParams DefaultParams = {
    PipelineParams::STUDENT,
    0,
    "data_1",
    0.2, 
    Eigen::Vector4f (-10.0, -5.0 ,-2.0, 1.0), 
    Eigen::Vector4f (30.0, 6.8, 1.0, 1.0), 
    30, 
    0.3, 
    0.4, 
    10, 
    500,
    false,
    false,
    false,
    true,
    true,
    true,
    false,
    0.2
};

static PipelineParams params = DefaultParams;

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

bool isBoxInsideSpecs(const Box & box, double tolerance)
{
// 1.58167x0.671334x1.3493
    double ref_w = 1.58;
    double ref_l = 0.67;
    double ref_h = 1.35;

    double w = box.x_max - box.x_min;
    double l = box.y_max - box.y_min;
    double h = box.z_max - box.z_min;

    bool w_ok = (w >= ref_w*(1.0 - tolerance)) && (w <= ref_w*(1.0 + tolerance));
    bool l_ok = (l >= ref_l*(1.0 - tolerance)) && (l <= ref_l*(1.0 + tolerance));
    bool h_ok = (h >= ref_h*(1.0 - tolerance)) && (h <= ref_h*(1.0 + tolerance));

    return w_ok && l_ok && h_ok;
}

double last_cyclist_angle = 0.0;
bool  cyclist_angle_found = false;

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    if( params.renderInputCloud )
        renderPointCloud(viewer,inputCloud,"inputCloud");

    //std::cout << "Filtering: cropMinPoint: " << params.cropMinPoint << std::endl;
    //std::cout << "Filtering: cropMaxPoint: " << params.cropMaxPoint << std::endl;

    // Experiment with the ? values and find what works best
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, params.filterResolution , params.cropMinPoint, params.cropMaxPoint);
    
    if( params.renderFilteredCloud )
    {
        renderPointCloud(viewer,filterCloud,"filterCloud");

        Box croppedArea;
        croppedArea.x_min = params.cropMinPoint.x();
        croppedArea.y_min = params.cropMinPoint.y();
        croppedArea.z_min = params.cropMinPoint.z();
        croppedArea.x_max = params.cropMaxPoint.x();
        croppedArea.y_max = params.cropMaxPoint.y();
        croppedArea.z_max = params.cropMaxPoint.z();
        renderBox(viewer, croppedArea, 100, Color(0,0,1), 0.5);
    }

    std::cout << "Filter Cloud has " << filterCloud->points.size () << " data points" << std::endl;
    if( filterCloud->points.size() == 0 )
    {
        return;
    }


    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, params.spMaxIterations, params.spDistanceThreshold);
    if( params.renderObstacles )
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));

    if (params.renderPlane)
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));


    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, params.clusterTolerance, params.clusterMinSize, params.clusterMaxSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};
    bool first_cyclist_found = false;

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size " << cluster->points.size() << std::endl;
        if(params.renderClusters)
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%5]);
        
        if(params.renderBoxes)
        {
            Box box = pointProcessorI->BoundingBox(cluster);

            if( params.followCyclist && !first_cyclist_found )
            {
                double w = box.x_max - box.x_min;
                double l = box.y_max - box.y_min;
                double h = box.z_max - box.z_min;

                pcl::PointXYZ box_center;
                box_center.x = box.x_min + w/2;
                box_center.y = box.y_min + l/2;
                box_center.z = box.z_min + h/2;

                std::cout << "Cluster dimensions: WxLxH = " << w << "x" << l << "x" << h << " at pos = " << box_center.x << ", " << box_center.y << ", " << box_center.z << std::endl;

                if( isBoxInsideSpecs(box, params.boxTolerance) )
                {

                    bool cyclist_found = false;

                    if( !cyclist_angle_found )
                    {
                        last_cyclist_angle = atan2(box_center.y,box_center.x);
                        cyclist_angle_found = true;
                        cyclist_found = true;
                    }
                    else
                    {
                        double cyclist_angle = atan2(box_center.y,box_center.x);
                        if( fabs(last_cyclist_angle - cyclist_angle) < 0.05 )
                        {
                            cyclist_found = true;
                            last_cyclist_angle = cyclist_angle;
                        }
                    }
                    

                    if(cyclist_found)
                    {
                        viewer->addLine<pcl::PointXYZ,pcl::PointXYZ>(pcl::PointXYZ(0,0,0), box_center, 1, 1, 1, "line"+std::to_string(clusterId));
                        renderBox(viewer, box, clusterId, Color(0,0,1));
                        first_cyclist_found = true;
                    }
                    else
                    {
                        renderBox(viewer, box, clusterId, Color(1,0,0));
                    }
                }
                else
                {
                    renderBox(viewer, box, clusterId, Color(1,0,0));
                }
                
            }
            else
            {
                renderBox(viewer, box, clusterId, Color(1,0,0));
            }
        }
        ++clusterId;
    }

    // mark that cyclist was not found, reset angle for next iteration
    if( !first_cyclist_found )
    {
        cyclist_angle_found = false;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    //ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    PclPointProcessor<pcl::PointXYZI> pointProcessorI;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    cityBlock( viewer, &pointProcessorI, inputCloud);

}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool renderObstacles = false;
    bool renderPlane = false;
    bool renderClusters = true;
    bool renderBoxes = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar * lidar = new Lidar(cars, 0.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  point_cloud = lidar->scan();

    //renderRays(viewer, lidar->position, point_cloud);
    //renderPointCloud(viewer, point_cloud, "PEPE");
    

    // TODO:: Create point processor
    PclPointProcessor<pcl::PointXYZ> point_processor;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = point_processor.SegmentPlane(point_cloud, 100, 0.2);
    if( renderObstacles )
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));

    if (renderPlane)
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));



    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        point_processor.numPoints(cluster);
        if(renderClusters)
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        
        if( renderBoxes )
        {
            Box box = point_processor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setSize(800,600);
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

void singleCityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}

void multipleCityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    using namespace std::chrono;

    PclPointProcessor<pcl::PointXYZI> pclPointProcessor;
    StudentPointProcessor<pcl::PointXYZI> studentPointProcessor;
    ProcessPointClouds<pcl::PointXYZI> * pointProcessorI = &pclPointProcessor;

    // Load params for the first time to see which data set should be used
    if( !paramsFilePath.empty() )
    {
        if( !params.fromFile(paramsFilePath) )
        {
            std::cerr << "Error reading params file" << std::endl;
        }
    }

    std::string base_path = "../src/sensors/data/pcd/";
    base_path.append(params.dataSet);

    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(base_path);
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    auto last = steady_clock::now();

    while (!viewer->wasStopped ())
    {
        auto now = steady_clock::now();

        // do not use sleep() for waiting, otherwise the viewer gets frozen between iterations!
        if( duration_cast<milliseconds>(now - last).count() > params.streamInterval_ms)
        {
            last = now;

            // Clear viewer
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            char msg[128];
            snprintf(msg, 128, "File: %d/%d", static_cast<int>(streamIterator - stream.begin() + 1), static_cast<int>(stream.size()));
            viewer->addText(msg, 10, 100, 25, 1,1,1);

            // Reload new params from file if user provided a params file
            if( !paramsFilePath.empty() )
            {
                if( !params.fromFile(paramsFilePath) )
                {
                    std::cerr << "Error reading params file" << std::endl;
                    continue;
                }
            }

            std::string processorTypeText;
            if( params.processorType == PipelineParams::PCL)
            {
                pointProcessorI = &pclPointProcessor;
                processorTypeText = "PCL";
            }
            else if( params.processorType == PipelineParams::STUDENT){
                pointProcessorI = &studentPointProcessor;
                processorTypeText = "STUDENT";
            }
            else { std::cerr << "Something went really wrong\n"; break;}

            snprintf(msg, 128, "Implementation: %s", processorTypeText.c_str());
            viewer->addText(msg, 10, 130, 25, 1,1,1);


            // Load pcd and run obstacle detection process
            inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
            std::cout << "==========================================================" << std::endl;

            auto start_pipeline = steady_clock::now();
            cityBlock(viewer, pointProcessorI, inputCloudI);
            auto end_pipeline = steady_clock::now();
            uint32_t elapsed_ms = duration_cast<milliseconds>(end_pipeline - start_pipeline).count();
            snprintf(msg, 128, "Time: %d ms", elapsed_ms);
            viewer->addText(msg, 10, 160, 25, 1,1,1);

            streamIterator++;
            if(streamIterator == stream.end())
            {
                streamIterator = stream.begin();
                cyclist_angle_found = false; // in case we are doing the cyclist challenge
            }

        }

        viewer->spinOnce ();
    }
}




int main (int argc, char** argv)
{

    if( argc > 1 )
    {
        paramsFilePath.assign(argv[1]);
    }

    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    //simpleHighway(viewer);
    //singleCityBlock(viewer);
    multipleCityBlock(viewer);
}

