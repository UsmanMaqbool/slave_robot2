/** @file pc_fetcher.h
 *	@brief pc_fetcher Subscribes to the topics from LSD-SLAM, publishes a pointcloud for pc_filter and republishes the LSD_SLAM topics with a header to allow synchronized listening (see central computer part)
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
// for image
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


using namespace std; 
using namespace sensor_msgs;


class pc_fetcher{
    public:

		//static bool keyframeTrigger;
		/*
		 * unique ID for each incoming keyframe
		 */
		static uint32_t kfIndex;
		/*
		 * variable used to get true pointcloud (taken from LSD-SLAM)
		 */
		static float scaledDepthVarTH;
		/*
		 * variable used to get true pointcloud (taken from LSD-SLAM)
		 */
		static int minNearSupport;

		pc_fetcher();
		~pc_fetcher();
		void pcCallback(slave_robot2::keyframeMsgConstPtr msg);
		void kfCallback(slave_robot2::keyframeGraphMsgConstPtr graph_msg);
	
    private:
		/*
		 * time stamp to republish LSD-SLAM messages as stamped messages
		 */
		ros::Time _timeStamp;

		/*
		 * Camera parameters
		 */
		float _fx, _fy, _cx, _cy, _fxi, _fyi, _cxi, _cyi;
		int _height, _width;
		/*
		 * struct to store image, depth values and statistics about depth values
		 */
		InputPointDense* _originalInput;

		/*
		 * Transformation from World (initial Robot COS) to camera COS (current Robot COS (trafo between camera and robot is assumed to be the identity trafo)
		 * 7 DOF similarity
		 * Rotation:
		 * _camToWorld[0], ..., _camToWorld[3]: quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()
		 * Translation:
		 * _camToWorld[4], ..., _camToWorld[6]: translation.x(), translation.y(), translation.z()
		 * Scale:
		 * quaternion.norm()
		 */
		Sophus::Sim3f _camToWorld;


		sensor_msgs::PointCloud _pointcloud;
		sensor_msgs::PointCloud2 _cloud2;
		slave_robot2::keyframeMsgStamped _kFMsgStamped;
		slave_robot2::keyframeGraphMsgStamped _kFGraphMsgStamped;

		ros::NodeHandle _nh;
		ros::Subscriber _pcSub;
		ros::Subscriber _kfSub;

		ros::Publisher _pcPub;
		ros::Publisher _kFMsgStampedPub;
		ros::Publisher _kFGraphMsgStampedPub;

		/*
		 * computes semi-dense pointcloud from depth values, depth values variances and camera parameters and stores them in the member variable _pointcloud
		 */
		void computeCloud();

		/*
		 * Reads the incoming keyframe Message from LSD-SLAM
		 */
		void readkFMsg(slave_robot2::keyframeMsgConstPtr msg);

		/*
		 * Fills in semi-dense point cloud to _cloud2
		 */
		void fillCloudMsg();

		/*
		 * Republishes incoming keyframe Message from LSD-SLAM as a stamped message to synchronize messages on central computer part
		 */
		void copykFMsgTokFMsgStamped(slave_robot2::keyframeMsgConstPtr msg);

		/*
		 * Republishes incoming keyframeGraph Message from LSD-SLAM as a stamped message to synchronize messages on central computer part
		 */
		void copykFGraphMsgTokFGraphMsgStamped(slave_robot2::keyframeGraphMsgConstPtr msg);

};

