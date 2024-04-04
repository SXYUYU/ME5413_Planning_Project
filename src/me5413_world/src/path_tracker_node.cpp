#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world
{

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_), lookahead_distance_(0.5), speed_target_(0.5)
{
    f_ = boost::bind(&PathTrackerNode::dynamicParamCallback, this, _1, _2);
    server_.setCallback(f_);

    sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
    sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

    world_frame_ = "world";
    robot_frame_ = "base_link";
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    odom_world_robot_ = *odom.get();
}

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
    if (!path->poses.empty()) {
        pose_world_goal_ = path->poses.back().pose;
        pub_cmd_vel_.publish(computeControlOutputs(odom_world_robot_, pose_world_goal_));
    }
}

void PathTrackerNode::dynamicParamCallback(const path_trackerConfig& config, uint32_t level)
{
    lookahead_distance_ = config.lookahead_distance;
    speed_target_ = config.speed_target;
}

double PathTrackerNode::computePurePursuitControl(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& goal_pose, double lookahead_distance)
{
    double steering_gain = 0.02;

    double dx = goal_pose.position.x - robot_pose.position.x;
    double dy = goal_pose.position.y - robot_pose.position.y;

    tf2::Quaternion q_robot_inv;
    tf2::fromMsg(robot_pose.orientation, q_robot_inv);
    q_robot_inv = q_robot_inv.inverse();
    tf2::Vector3 delta_pos_world(dx, dy, 0);
    tf2::Vector3 delta_pos_robot = tf2::quatRotate(q_robot_inv, delta_pos_world);

    double X = delta_pos_robot.x();
    double Y = delta_pos_robot.y();
    double steering_angle = atan2(2 * Y * steering_gain, lookahead_distance);

    return steering_angle;
}

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
{
    double steering_angle = computePurePursuitControl(odom_robot.pose.pose, pose_goal, lookahead_distance_);

    double dx = pose_goal.position.x - odom_robot.pose.pose.position.x;
    double dy = pose_goal.position.y - odom_robot.pose.pose.position.y;
    double distance_to_goal = sqrt(dx*dx + dy*dy);

    double speed;
    if (distance_to_goal > lookahead_distance_) {
        speed = std::min(distance_to_goal / 2, speed_target_); // Increase speed based on distance
    } else {
        speed = std::max(speed_target_ * distance_to_goal / lookahead_distance_, speed_target_ * 0.8); // Reduce speed as it gets closer
    }

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = speed;
    cmd_vel.angular.z = steering_angle;

    return cmd_vel;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_tracker_node");
    me5413_world::PathTrackerNode path_tracker_node;
    ros::spin();
    return 0;
}

