/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include "iostream"
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker : What do you mean??
#include "processPointClouds.cpp"

#include "quiz/ransac/kdtree.h"
//#include "quiz/ransac/ransac3d.cpp"
#include <unordered_set>

/*
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    
    // 'Car' struct is in render.h
    
    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar"); //Make struct instance
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars; //vector which has type <Car> puch_back 'Car' instance.
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer); //renderHighway function is in 'render.cpp'.
        egoCar.render(viewer); 
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars; //and, return this vector
}
*/

/*
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    bool render_obst = false;
    bool render_plane = false;
    bool render_clusters = true;
    bool render_box = true;

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer); // copy vector
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0); //Lidar struct allocation which has 'vector car' as input argument. 'lidar' is Lidar struct pointer.
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan(); //'lidar' pointer call lidar.h --> scan() method. Output pointcloud data type is always 'pcl::PointCloud<pcl::PointXYZ>::Ptr'
    //renderRays(viewer, lidar->position, inputCloud); 
    //renderPointCloud(viewer, inputCloud, "inputCloud"); //renser.cpp --> renderPointCloud

    ProcessPointClouds<pcl::PointXYZ> pointProcessor; 

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedPoint = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    //segmentedPoint = processPointClouds->SegmentPlane(inputCloud, 1000, 0.01);
    if(render_obst)
        renderPointCloud(viewer, segmentedPoint.first, "obstCloud", Color(1, 0, 0));
    if(render_plane)
        renderPointCloud(viewer, segmentedPoint.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentedPoint.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    
    // renderPointCloud is expecting each pcl viewer point cloud to have unique identifier.
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters){
        if(render_clusters){
            std::cout<<"Cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
        }
        if(render_box){
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
}
*/

/*
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer){
    // ----------------------------------------------------
    // -----Open 3D viewer and display city blocks -----
    // ----------------------------------------------------
    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000001.pcd");
     
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor->FilterCloud(inputCloud, 0.1, Eigen::Vector4f(.1,.1,.1,1), Eigen::Vector4f(100,100,100,1));
    renderPointCloud(viewer, filteredCloud, "inputCloud");

}
*/

void Proximity(int idx, const std::vector<std::vector<float>> &points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float dT){
	processed[idx] = true;
	cluster.push_back(idx);
	std::vector<int> nearby = tree->search(points[idx], dT);

	for(int id : nearby){
		if(processed[id] != true){
			Proximity(id, points, cluster, processed, tree, dT);//This recustion takes 2D points, 1D cluster, 1D processed, *tree, dT
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(std::vector<std::vector<float>> points, KdTree *tree, float distanceTol) {
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);//all vector element is initialized by 'false'.
	
		for(int i=0; i<points.size(); i++){
			if( processed[i] != true ){
				std::vector<int> cluster;
				Proximity(i, points, cluster, processed, tree, distanceTol);
				clusters.push_back(cluster);
			}
		}
	return clusters;
}


std::unordered_set<int> RansacI(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	pcl::PointXYZ max_p1;
	pcl::PointXYZ max_p2;
	pcl::PointXYZ max_p3; //for 3D ransac
	int _max = 0;

	while(maxIterations--){
		// Randomly sample subset and fit line
		pcl::PointXYZ p1 = cloud->points[rand()%(cloud->points.size())];
		pcl::PointXYZ p2 = cloud->points[rand()%(cloud->points.size())];
		pcl::PointXYZ p3 = cloud->points[rand()%(cloud->points.size())]; //for 3D ransac
		
		float i = (p2.y-p1.y)*(p3.z-p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		float j = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x-p1.x)*(p3.x-p1.z);
		float k = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);
		float D = (-1)*(i*p1.x + j*p1.y + k*p1.z);

		int cnt = 0;
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// Measure distance between every point and fitted line
			pcl::PointXYZ sample = cloud->points[index];

			// Sample point should not be included in 'line making points(p1, p2)'.
			if( (sample.x == p1.x && sample.y == p1.y && sample.z == p1.z)|| (sample.x == p2.x && sample.y == p2.y && sample.z == p2.z) || (sample.x == p3.x && sample.y == p3.y && sample.z == p3.z) )
				continue;

			float distance3D = fabs(i*sample.x + j*sample.y + k*sample.z + D) / sqrt(pow(i,2) + pow(j,2) + pow(k,2));

			// If distance is smaller than threshold count it as inlier
			if(distance3D <= distanceTol){
				cnt++;
			}
		}
		if(cnt > _max){
			max_p1 = p1; //pcl::PointXYZ support = operation overload.
			max_p2 = p2;
			max_p3 = p3;
			_max = cnt;
		}
	}
	float i = (max_p2.y-max_p1.y)*(max_p3.z-max_p1.z) - (max_p2.z-max_p1.z)*(max_p3.y-max_p1.y);
	float j = (max_p2.z-max_p1.z)*(max_p3.x-max_p1.x) - (max_p2.x-max_p1.x)*(max_p3.x-max_p1.z);
	float k = (max_p2.x-max_p1.x)*(max_p3.y-max_p1.y) - (max_p2.y-max_p1.y)*(max_p3.x-max_p1.x);
	float D = (-1)*(i*max_p1.x + j*max_p1.y + k*max_p1.z);

	for(int index = 0; index < cloud->points.size();index++){
		pcl::PointXYZ sample = cloud->points[index];
		float distance3D = fabs(i*sample.x + j*sample.y + k*sample.z + D) / sqrt(pow(i,2) + pow(j,2) + pow(k,2));
	
		if(distance3D <= distanceTol){
			inliersResult.insert(index);
		}
	}

	return inliersResult;
}


void cityBlock_stream(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZ> *pointProcessor, const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud){
    /*
        Just by using const, you are not acually changing the 'inputCloud'.
        const reference is better memory efficiency. Cause, you just read from it.
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = pointProcessor->FilterCloud(inputCloud, 0.25, Eigen::Vector4f(-20, -5, -3, 1), Eigen::Vector4f(20,7,20,1));

    bool render_clusters = true;
    bool render_box = true;
    //--------- HyperParamters -----------//
    int maxIterations = 200;
    float distanceTol = 0.5;

    //------------------------------------//

    // PCL's built segmentaion
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedPoint = pointProcessor->SegmentPlane(filteredCloud, 100, 0.2);
    //renderPointCloud(viewer, segmentedPoint.first, "obstCloud", Color(1, 0, 0));
    //renderPointCloud(viewer, segmentedPoint.second, "planeCloud", Color(0, 1, 0));

    // Project: Ransac segmentation
    std::unordered_set<int> inliers = RansacI(filteredCloud, maxIterations, distanceTol);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < filteredCloud->points.size(); index++) {
		pcl::PointXYZ point = filteredCloud->points[index];
		if(inliers.count(index)) // Check whether index is element of inliers.
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	renderPointCloud(viewer, cloudInliers,"inliers",Color(0,1,0));
    //renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));

    // PCL's build clustering    
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentedPoint.first, 1.0, 3, 30);
    // int clusterId = 0;
    // std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    
    //  for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters){
    //      if(render_clusters){
    //          std::cout<<"Cluster size ";
    //          pointProcessor.numPoints(cluster);
    //          renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
    //      }
    //      if(render_box){
    //          Box box = pointProcessor.BoundingBox(cluster);
    //          renderBox(viewer, box, clusterId);
    //      }
    //      ++clusterId;
    // }


    // Project: Euclidean clustering 
    // Before we do, we need to convert all 'pcl::PointCloud<pcl::PointXYZ>::Ptr cloud->points' to 
	// std::vector<std::vector<float>> points to use euclidean clustring algorithm.   
    
    //[TODO] Remove below and make it more efficient.----------------//
    std::vector<std::vector<float>> points_input;
	for(int i=0;i<cloudOutliers->points.size();i++){
		points_input.push_back({cloudOutliers->points[i].x, 
								cloudOutliers->points[i].y,
								cloudOutliers->points[i].z});	
	}
    //--------------------------------------------------------------//

	KdTree *tree = new KdTree;
	for(int i=0; i< cloudOutliers->points.size();i++){
		tree->insert(points_input[i],i);
        //tree->insert(cloudOutliers->points[i],i);
	}

	std::vector<std::vector<int>> cloudClusters = euclideanCluster(points_input, tree, 0.5);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,0,1)};//, Color(0,1,1), Color(1,1,0), Color(1,0,1)};
    for (std::vector<int> cluster : cloudClusters){
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (int indice: cluster){
            clusterCloud->points.push_back(pcl::PointXYZ(points_input[indice][0], 
                                                          points_input[indice][1], 
                                                          points_input[indice][2]));
        }
        renderPointCloud(viewer, clusterCloud, "cluster"+std::to_string(clusterId), colors[clusterId%3]);
        Box box = pointProcessor->BoundingBox(clusterCloud);
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

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer")); //What is the grammer inside??
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1");//put all pcd files in vector
    auto streamIterator = stream.begin(); // declare itertor.
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;

    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //Load pcd and run obstacle detection process
        inputCloud = pointProcessor->loadPcd((*streamIterator).string()); //Load one pcd file.
        cityBlock_stream(viewer, pointProcessor, inputCloud); //loaded cloud.

        streamIterator++; //This iterator take another file.
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();

        // int key = cv::waitKey(0);
		// if (key == 32)
		// 	break;
        //system("PAUSE");
        
        //cin.get();
    } 
}