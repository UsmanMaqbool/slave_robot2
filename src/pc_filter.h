/** @file pc_filter.h
 *	@brief pc_filter Subscribes to the pointcloud from pc_fetcher, does random sampling and applies and outlier removal filter. Finally publisheds the filtered pointcloud.
 *
 *	@author Tzi Yang Shao
 *	@author Daniel Schneider
*/


#include "KeyFrameDisplay.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include <stdint.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>

class pc_filter{
    public:

		/*
		 * To what extend the Point Cloud should be reduced. rS times less points will be kept. In other words, a fraction of 1/rS points are kept after the random sampling filter
		 */
		float randSampling;

		/*
		 * Standard Deviation multiplier for the statistical outlier removal filter.
		 * The higher, the more points are filtered out.
		 * All points who have a distance larger than outlierRemStdDev*stddev from the mean distance to the query point will be marked as outliers and removed
		 */
		float outlierRemStdDev;

		/*
		 * The number of neighbors considered to analyze the statistics for each point
		 */
		int nbrNeigh;

		/*
		 * @brief Constructor creating a ros node, subscribing to the pointcloud from pc_fetcher, filtering, and publishing the filtered pointcloud
		 * @param rS To what extend the Point Cloud should be reduced. rS times less points will be kept. In other words, a fraction of 1/rS points are kept after the random sampling filter
		 * @param oR Standard Deviation multiplier for the statistical outlier removal filter.
		 * The higher, the more points are filtered out.
		 * All points who have a distance larger than oR*stddev from the mean distance
		 *  to the query point will be marked as outliers and removed
		 * @param nN The number of neighbors considered to analyze the statistics for each point
		 */
		pc_filter(float rS=5.0, float oR=0.00001, int nN = 15);

		/*
		 * @brief Default destructor
		 */
		~pc_filter();

		/*
		 * @brief Processes the incoming pointcloud (random sampling and outlier removal)
		 */
		void pcCallback(const sensor_msgs::PointCloud2ConstPtr & cloud2);
	
    private:
		ros::NodeHandle _slaveRobot;
		ros::Subscriber _pcSub;
		ros::Publisher _pcPub;
};
