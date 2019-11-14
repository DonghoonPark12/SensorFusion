/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#include <iostream>
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include "kdtree.h"

#define TwoDim //ThreeDim

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

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	/* Ransac 2D */
	pcl::PointXYZ max_p1;
	pcl::PointXYZ max_p2;
	int _max = 0;
	
	while(maxIterations--){
		// Randomly sample subset and fit line
		pcl::PointXYZ p1 = cloud->points[rand()%(cloud->points.size())];
		pcl::PointXYZ p2 = cloud->points[rand()%(cloud->points.size())];
		
		int cnt = 0;
		for(int index = 0; index < cloud->points.size(); index++){
			// Measure distance between every point and fitted line
			pcl::PointXYZ sample = cloud->points[index];

			// Sample point should not be included in 'line making points(p1, p2)'.
			if( (sample.x == p1.x && sample.y == p1.y)|| (sample.x == p2.x && sample.y == p2.y) )
				continue;

			float distance2D = fabs((p1.y - p2.y)*sample.x + (p2.x - p1.x)*sample.y + (p1.x*p2.y - p2.x*p1.y))/sqrt(pow(p1.y - p2.y,2) + pow(p2.x - p1.x,2));

			// If distance is smaller than threshold count it as inlier
			if(distance2D <= distanceTol){
				cnt++;
			}
		}

		if(cnt > _max){
			max_p1.x = p1.x; max_p1.y = p1.y;
			max_p2.x = p2.x; max_p2.y = p2.y;
			_max = cnt;
			cout<<_max<<endl;
		}
	}

	for(int index = 0; index < cloud->points.size();index++){
		pcl::PointXYZ sample = cloud->points[index];
		float distance2D = fabs((max_p1.y - max_p2.y)*sample.x + (max_p2.x - max_p1.x)*sample.y + (max_p1.x * max_p2.y - max_p2.x * max_p1.y))/sqrt(pow(max_p1.y - max_p2.y,2) + pow(max_p2.x - max_p1.x,2));
	
		if(distance2D <= distanceTol){
			inliersResult.insert(index);
		}
	}
	cout<<inliersResult.size()<<endl;
		 
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	/* Ransac 3D */
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
		for(int index = 0; index < cloud->points.size(); index++){
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
			cout<<_max<<endl;
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
	cout<<inliersResult.size()<<endl;
	
	return inliersResult;
}

void Proximity(int idx, const std::vector<std::vector<float>> &points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float dT){
	processed[idx] = true;
	cluster.push_back(idx);
	std::vector<int> nearby = tree->search(points[idx], dT);
	//for(int i=0; i<nearby.size(); i++){
	for(int id : nearby){
		if(processed[id] != true){
			Proximity(id, points, cluster, processed, tree, dT);//This recustion takes 2D points, 1D cluster, 1D processed, *tree, dT
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> points, KdTree *tree, float distanceTol) {
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;

	//int *check = new (int) * points.size();
	std::vector<bool> processed(points.size(), false);//all vector element is initialized by 'false'.
	
	//for (int i=0;i<clusters.size(); i++){
		for(int i=0; i<points.size(); i++){
			if( processed[i] != true ){
				//check[j] = 1;
  				//pcl::PointCloud<pcl::PointXYZ>::Ptr one_cluster(new pcl::PointCloud<pcl::PointXYZ>());//allocation cluster
				std::vector<int> cluster;

				//std::vector<int> nearby = tree->search(points[i], distanceTol);
				Proximity(i, points, cluster, processed, tree, distanceTol);

				// for(int indice: one_cluster)// There would be 3 clusters.
				// 	clusters->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
				clusters.push_back(cluster);
			}
		}
	//}
	return clusters;
}


int main (){

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	#ifdef TwoDim
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	#else 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	#endif

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// unordered_set is container that store unique elements in no paricular order.
	#ifdef TwoDim
		std::unordered_set<int> inliers = Ransac2D(cloud, 100, 0.5);
	#else
		std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.5);
	#endif

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index)) // Check whether index is element of inliers.
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// ----------------------------- Insert 3D points in KdTree ------------------------------ //
	// Before we do, we need to convert all 'pcl::PointCloud<pcl::PointXYZ>::Ptr cloud->points' to 
	// std::vector<std::vector<float>> points to use euclidean clustring algorithm.

	// std::vector<std::vector<float>> points_input;
	// for(int i=0;i<cloudOutliers->points.size();i++){
	// 	points_input.push_back({cloudOutliers->points[i].x, 
	// 							cloudOutliers->points[i].y,
	// 							cloudOutliers->points[i].z});	
	// }

	// KdTree *tree = new KdTree;
	// for(int i=0; i< cloudOutliers->points.size();i++){
	// 	tree->insert(points_input[i],i);
	// }
	// //for(int i=0;i<cloudOutliers->points.size();i++){//insert 3D points to KdTree
	// //	tree->insert(vp, i);
	// //}
	// std::vector<std::vector<int>> clusters = euclideanCluster(points_input, tree, 3.0);

	// -------------------------------------------------------------------------------------- //
	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));

		// int clusterId = 0;
		// std::vector<Color> colors = {Color(1,0,0), Color(0,0,1)};
		// for (std::vector<int> cluster : clusters){
		// 	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
		// 	for (int indice: cluster){
		// 		clusterCloud->points.push_back(pcl::PointXYZ(points_input[indice][0], \
		// 									  				 points_input[indice][1], \
		// 									 				 points_input[indice][2]));
		// 	}
		// 	renderPointCloud(viewer, clusterCloud, "cluster"+std::to_string(clusterId), colors[clusterId%2]);
		// 	++clusterId;
		// }
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
