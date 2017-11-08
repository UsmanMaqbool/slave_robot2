#include "pc_filter.h"
//filter used
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>


using namespace std; 
using namespace sensor_msgs;


pc_filter::pc_filter(float rS, float oR, int nN) : randSampling(rS), outlierRemStdDev(oR), nbrNeigh(nN) {
	_pcSub = _slaveRobot.subscribe("/slave_robot2/pointcloud2", 5, &pc_filter::pcCallback, this);
	_pcPub = _slaveRobot.advertise<sensor_msgs::PointCloud2>("/slave_robot2/pointcloud2Filtered", 5);
}
pc_filter::~pc_filter(){
	//destructor to release the memory
}

void pc_filter::pcCallback(const sensor_msgs::PointCloud2ConstPtr & cloud_msg)
{ 
// Container for original & filtered data
  sensor_msgs::PointCloud output;
  sensor_msgs::PointCloud2 output_pc;
  geometry_msgs::Point32 _tempbuffer;
  int numberSampled = (cloud_msg->width)/this->randSampling;



  pcl::PointCloud<pcl::PointXYZ>::Ptr PC (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr random_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr PC_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *PC);

  pcl::RandomSample<pcl::PointXYZ>ransamp;
  ransamp.setInputCloud (PC);
  ransamp.setSample (numberSampled);
  ransamp.setSeed(rand());
  //ransamp.setKeepOrganized(true);
  ransamp.filter(*random_filtered);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_rm;
  outlier_rm.setInputCloud (random_filtered);
  outlier_rm.setMeanK(this->nbrNeigh);
  outlier_rm.setStddevMulThresh (this->outlierRemStdDev);
// The resulting cloud_out contains all points of cloud_in that have an average distance to their 8 nearest neighbors that is below the computed threshold
// Using a standard deviation multiplier of 1.0 and assuming the average distances are normally distributed there is a 84.1% chance that a point will be an inlier
  //outlier_rm.setKeepOrganized(true);
  outlier_rm.filter (*PC_filtered);

  // Convert to ROS data type
  output.header.frame_id="world";
  output.header.stamp = cloud_msg->header.stamp;
  output.header.seq = cloud_msg->header.seq;
for (size_t i = 0; i < PC_filtered->points.size (); ++i){
	_tempbuffer.x=PC_filtered->points[i].x;
	_tempbuffer.y=PC_filtered->points[i].y;
	_tempbuffer.z=PC_filtered->points[i].z;
	output.points.push_back(_tempbuffer);
}
	
  sensor_msgs::convertPointCloudToPointCloud2(output,output_pc);

  // Publish the data
  _pcPub.publish (output_pc);


}

