#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main (int argc, char **argv)
{
	ros::init (argc, argv, "pub_pcl");
	ros::NodeHandle nh;
	ros::Publisher pcl_pub = nh.advertise<PointCloud> ("pcl_output", 1);

	sensor_msgs::PointCloud2 output;
	pcl::PointCloud<pcl::PointXYZ> cloud;

    //Load test.pcd file
	pcl::io::loadPCDFile ("//home//jon//Downloads//room_scan1.pcd", cloud);

	pcl::toROSMsg (cloud, output);
	output.header.frame_id = "point_cloud";

	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		pcl_pub.publish (output);
		ros::spinOnce ();
		loop_rate.sleep ();
	}

	return 0;
}

