/** @file slave_robot.h
 *	@brief Main program running on the slave robot. Creates instances of listener and publisher classes. Defining necessary parameters
 *
 *	@author Tzi Yang Shao
 *	@author Daniel Schneider
*/

#include "pc_fetcher.h"
#include "pc_filter.h"

using namespace std; 
using namespace sensor_msgs;
uint32_t pc_fetcher::kfIndex=(uint32_t)0;
float pc_fetcher::scaledDepthVarTH = 1;
int pc_fetcher::minNearSupport = 5;

int main(int argc, char **argv){
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listening_keyframe");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   *
   * Can modify the 3 parameters randomSampling, statOutlRemStdDev and numberNeighbors to change the filtering behavier
   *
   * randomSampling: To what extend the Point Cloud should be reduced. rS times less points will be kept. In other words, a fraction of 1/rS points are kept after the random sampling filter
   *
   * statOutlRemStdDev: Standard Deviation multiplier for the statistical outlier removal filter.
   * 					The higher, the more points are filtered out.
   * 					All points who have a distance larger than oR*stddev from the mean distance to the query point will be marked as outliers and removed
   *
   * numberNeighbors: The number of neighbors considered to analyze the statistics for each point
   */
  pc_fetcher fetch;
  float randomSampling = 10.0;
  float statOutlRemStdDev = 0.0000001;
  int numberNeighbors = 50;
  pc_filter filter(randomSampling, statOutlRemStdDev, numberNeighbors);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
   ros::spin();
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  return 0;
}
