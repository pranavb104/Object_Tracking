// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

//To Prevent Warnings
#ifdef _DEBUG
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <iostream>
#include <sstream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

//To make code shorter ;D
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

	

//Callback pointcloud
pcl_ptr points_to_pcl(const rs2::points& points)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}

int main(int argc, char* argv[])
{
	//PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
		new pcl::visualization::PCLVisualizer(argc, argv, "Point Cloud Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);


	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	
	//U can add configure pipelines to change the input resolution of your feed.
	
	// Start streaming with default recommended configuration
	pipe.start();

	float c_x, c_y, c_z;


	while (!viewer->wasStopped()) // Application still alive?
	{
		viewer->spinOnce();

		// Wait for the next set of frames from the camera
		auto frames = pipe.wait_for_frames();

		auto depth = frames.get_depth_frame();

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		auto pcl_points = points_to_pcl(points);
		
		/* Processing Point Cloud starts here */
		
		//Reduce point cloud resolution for faster processing
		pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(pcl_points);
		sor.setLeafSize(0.01f, 0.01f, 0.01f);
		sor.filter(*cloud_filtered);

		if (!viewer->updatePointCloud(cloud_filtered, "cloud_filtered"))
		{
			viewer->addPointCloud(cloud_filtered, "cloud_filtered");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.2, 1.0, "cloud_filtered");

		}

		// Filtering Box (XYZ)
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud_filtered);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.6, 1.0);
		pass.filter(*cloud_t);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_g(new pcl::PointCloud<pcl::PointXYZ>());
		pass.setInputCloud(cloud_t);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(-0.15, 0.15);
		pass.filter(*cloud_g);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>());
		pass.setInputCloud(cloud_g);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-0.4, 0.4);
		pass.filter(*cloud_p);

		// Estimate point normals
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		ne.setInputCloud(cloud_p);
		ne.setKSearch(50);
		ne.compute(*cloud_normals);

		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
		seg.setNormalDistanceWeight(0.1);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.04);
		seg.setInputCloud(cloud_p);
		seg.setInputNormals(cloud_normals);

		// Obtain the plane inliers and coefficients
		pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
		seg.segment(*inliers_plane, *coefficients_plane);
		//std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_p);
		extract.setIndices(inliers_plane);
		extract.setNegative(false);

		// Write the planar inliers to disk
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
		extract.filter(*cloud_plane);
		if (!viewer->updatePointCloud(cloud_plane, "cloud_plane"))
		{
			viewer->addPointCloud(cloud_plane, "cloud_plane");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.8, 0.2, "cloud_plane");

		}


		// Remove the planar inliers, extract the rest
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		extract.setNegative(true);
		extract.filter(*cloud_filtered2);
		extract_normals.setNegative(true);
		extract_normals.setInputCloud(cloud_normals);
		extract_normals.setIndices(inliers_plane);
		extract_normals.filter(*cloud_normals2);

		// Create the segmentation object for cylinder segmentation and set all the parameters
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CIRCLE3D);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight(0.1);
		seg.setMaxIterations(10000);
		seg.setDistanceThreshold(0.1);
		seg.setRadiusLimits(0, 0.4);
		seg.setInputCloud(cloud_filtered2);
		seg.setInputNormals(cloud_normals2);

		// Obtain the cylinder inliers and coefficients
		seg.segment(*inliers_cylinder, *coefficients_cylinder);

		// Write the cylinder inliers to disk
		extract.setInputCloud(cloud_filtered2);
		extract.setIndices(inliers_cylinder);
		extract.setNegative(false);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
		extract.filter(*cloud_cylinder);
		if (cloud_cylinder->points.empty())
			std::cerr << "Can't find the cylindrical component." << std::endl;

		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr treee(new pcl::search::KdTree<pcl::PointXYZ>);
		treee->setInputCloud(cloud_cylinder);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.02); // 2cm
		ec.setMinClusterSize(35);  	//Manipulate min and max cluster to control object size
		ec.setMaxClusterSize(110);
		ec.setSearchMethod(treee);
		ec.setInputCloud(cloud_cylinder);
		ec.extract(cluster_indices);

		//Use this to check Cluster Size
		//std::cout << "Number of clusters is equal to " << cluster_indices.size() << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>());

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			cloud_cluster->clear();

			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
				cloud_cluster->points.push_back(cloud_cylinder->points[*pit]);
			//std::cout << " Number of points in object : " << cloud_cluster->size() << std::endl;
		}
		
		//Checking Eigen coordinates of resulting PointCloud
		Eigen::Vector4f pcaCentroid;
		pcl::compute3DCentroid(*cloud_cluster, pcaCentroid);
		Eigen::Matrix3f covariance;
		computeCovarianceMatrixNormalized(*cloud_cluster, pcaCentroid, covariance);
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
		Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
		eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

		// Transform the original cloud to the origin where the principal components correspond to the axes.
		Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
		projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
		projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*cloud_cluster, *cloudPointsProjected, projectionTransform);
		// Get the minimum and maximum points of the transformed cloud.
		pcl::PointXYZ minP, maxP;
		pcl::getMinMax3D(*cloudPointsProjected, minP, maxP);
		Eigen::Vector3f meanDiagonal = 0.5f*(maxP.getVector3fMap() + minP.getVector3fMap());


		c_x = pcaCentroid.x() * 1000;
		c_y = pcaCentroid.y() * 1000;
		c_z = pcaCentroid.z() * 1000;
		
		//Coordinates of resulting object
		//std::cout << "x = " << (c_x) << "mm  y = " << (c_y) << "mm z = " << (c_z) << "mm" << std::endl;

		/* Processing Point Cloud ends here */

		// Final transform
		Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
		Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
		viewer->removeAllShapes();
		viewer->addCube(bboxTransform, bboxQuaternion, maxP.x - minP.x, maxP.y - minP.y, maxP.z - minP.z, "bbox");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7, 0.7, 0, "bbox");
		viewer->setRepresentationToWireframeForAllActors();
		viewer->setShowFPS(false);

		// Update Point Cloud
		if (!viewer->updatePointCloud(cloud_cluster, "cloud_cluster"))
		{
			viewer->addPointCloud(cloud_cluster, "cloud_cluster");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.2, "cloud_cluster");
		}
	}

	
	return EXIT_SUCCESS;

}

