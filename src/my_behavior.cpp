#include "my_behavior.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <laser_geometry/laser_geometry.h>

//#include "nodes/check_obstacle.hpp"
//#include "nodes/drive_action.hpp"

namespace zmp
{
Rc110Behavior::Rc110Behavior(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate) :
       // parameters({.treeFile = handlePrivate.param<std::string>("tree_file", "tree.xml")}),
        laserSubscriber(handle.subscribe("scan", 1, &Rc110Behavior::onLaser, this)),
        drivePublisher(handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive_ad", 1)),
        startTime(ros::Time::now())
{/*
    registerNodeBuilder<CheckObstacle>(std::cref(cloud));
    registerNodeBuilder<DriveAction>(std::ref(drivePublisher));

    try {
        //behaviorTree = behaviorTreeFactory.createTreeFromFile(parameters.treeFile);
    } catch (std::runtime_error& e) {
        throw std::runtime_error("Unable to find tree file in: " + parameters.treeFile);
    }
}

void Rc110Behavior::update()
{
    if (cloud.data.empty() && ros::Time::now() - startTime < ros::Duration(1)) {
        // waiting for point cloud 1 sec
        return;
    }
  if (behaviorTree.tickRoot() == BT::NodeStatus::FAILURE) {
        ROS_INFO_THROTTLE(10, "Behavior tree failed");
    }*/
}

void Rc110Behavior::onLaser(const sensor_msgs::LaserScan& scan)
{
    laser_geometry::LaserProjection projection;
    projection.projectLaser(scan, cloud);

    for (Iter it = {cloud, "x"}; it != it.end(); ++it) {
        float x = it[0], y = it[1];
        Eigen::Vector2f point = {x, -y};  // flip along x axis (temporary solution)

        if (nearBox.contains(point)) {
            result = "near";
            break;
        } else if (farLeftBox.contains(point)) {
            result = "far";
            closestLeftX = std::min(x, closestLeftX);
        } else if (farRightBox.contains(point)) {
            result = "far";
            closestRightX = std::min(x, closestRightX);
        }
    }

    ackermann_msgs::AckermannDriveStamped cmdMsg;
    cmdMsg.drive.speed = speed.value();
    cmdMsg.drive.steering_angle = static_cast<float>(steering.value()) * DEG_TO_RAD;

    drivePublisher.publish(cmdMsg);
}

}  // namespace zmp
