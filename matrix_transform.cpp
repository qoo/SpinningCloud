#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
auto Transformation(const PointCloud<pcl::PointXYZ>::ConstPtr &cloud)->PointCloud<pcl::PointXYZ>::ConstPtr;
void Visualization(const PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
int main()
{
	// Load file | Works with PCD and PLY files
	Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
	cout << transform_x;
	cout << "hello";
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("ism_test_cat.pcd", *source_cloud);
	Visualization(source_cloud);
	return 0;
}
auto Transformation(const PointCloud<pcl::PointXYZ>::ConstPtr &cloud)->PointCloud<pcl::PointXYZ>::ConstPtr
{
	static int thic{};
	float theta{};

	thic++;
	theta = (thic % 100)*(M_PI / 50);
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	//float theta = M_PI / 4; // The angle of rotation in radians
	transform_1(0, 0) = cos(theta);
	transform_1(0, 1) = -sin(theta);
	transform_1(1, 0) = sin(theta);
	transform_1(1, 1) = cos(theta);
	//    (row, column)

	// Define a translation of 2.5 meters on the x axis.
	transform_1(0, 3) = 2.5;


	/*  METHOD #2: Using a Affine3f
	This method is easier and less error prone
	*/


	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_1);
	return transformed_cloud;

}
void Visualization(const PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{

	pcl::visualization::PCLVisualizer viewer("Spinning Cloud");


	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "spinning_cloud");
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		PointCloud<pcl::PointXYZ>::ConstPtr t_cloud = Transformation(cloud);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(t_cloud, 230, 20, 20); // Red
		viewer.addPointCloud(t_cloud, transformed_cloud_color_handler, "spinning_cloud");
		viewer.spinOnce();
		viewer.removePointCloud("spinning_cloud");
	}

}