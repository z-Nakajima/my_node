#pragma once

//#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace zmp
{
/**
 * Main car controlling class.
 * It's reading sensors and moving drives accordingly.
 */
class Rc110Behavior
{
    struct Parameters {
        std::string treeFile;
    };

public:
    Rc110Behavior(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate);

    /**
     * Behavior tree tick that is triggered in the main loop.
     */
    //void update();

private:


    void onLaser(const sensor_msgs::LaserScan& scan);

private:
    Parameters parameters;
    ros::Subscriber laserSubscriber;
    ros::Publisher drivePublisher;

   // BT::BehaviorTreeFactory behaviorTreeFactory;
   // BT::Tree behaviorTree;

    sensor_msgs::PointCloud2 cloud;
    ros::Time startTime;
};
}  // namespace zmp
