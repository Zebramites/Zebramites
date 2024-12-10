#include <cmath>

#include <ros/ros.h>
#include <angles/angles.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <hardware_interface/joint_command_interface.h>


namespace mecanum_drive_controller
{

class MecanumDriveController
	: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:

hardware_interface::JointHandle fl_wheel_joint_;
hardware_interface::JointHandle fr_wheel_joint_;
hardware_interface::JointHandle bl_wheel_joint_;
hardware_interface::JointHandle br_wheel_joint_;

realtime_tools::RealtimeBuffer<geometry_msgs::Twist> command_;

ros::Subscriber sub_command_;

bool init(hardware_interface::VelocityJointInterface *hw,
		ros::NodeHandle &/*root_nh*/,
		ros::NodeHandle &controller_nh)
{
    
    fl_wheel_joint_ = hw->getHandle("fl_drive");
    fr_wheel_joint_ = hw->getHandle("fr_drive");
    bl_wheel_joint_ = hw->getHandle("bl_drive");
    br_wheel_joint_ = hw->getHandle("br_drive");

	const std::string complete_ns = controller_nh.getNamespace();

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &MecanumDriveController::cmdVelCallback, this);
	
	return true;
}

void starting(const ros::Time &time)
{
    
}

void stopping(const ros::Time & time)
{

}

double sign(double x)
{
    return (x > 0) - (x < 0);
}

double remap(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update(const ros::Time &time, const ros::Duration &period)
{
    ros::spinOnce();
    
	// compute mecanum drive velocities
    const geometry_msgs::Twist curr_cmd = *(command_.readFromRT());
    
    double y = -curr_cmd.linear.y;
    double x = curr_cmd.linear.x;
    double rx = -curr_cmd.angular.z;
    //ROS_INFO_STREAM_THROTTLE(5, "Y " << y << " X " << x  << " Z " << rx);

    fl_wheel_joint_.setCommand(y + x + rx);
    bl_wheel_joint_.setCommand(y - x + rx);
    fr_wheel_joint_.setCommand(y - x - rx);
    br_wheel_joint_.setCommand(y + x - rx);

    return;
    // stop here
    // double angle = atan2(curr_cmd.linear.y, curr_cmd.linear.x);
    // angle *= (4 / M_PI);
    // angle = round(angle);
    // angle += 4;

    // double frontLeftPower = rx;
    // double backLeftPower = rx;
    // double frontRightPower = -rx;
    // double backRightPower = -rx;

    // double stickMagnitude = std::sqrt(std::pow(y, 2) + std::pow(x, 2));
    // double magnitude = stickMagnitude > 0 ? stickMagnitude : 1;

    // if (std::sqrt(std::pow(y, 2) + std::pow(x, 2)) > 0.2) {

    //     if (angle == 0 || angle == 8) {
    //         // move to the right
    //         frontLeftPower = 1;
    //         backLeftPower = -1;
    //         frontRightPower = -1;
    //         backRightPower = 1;
    //     }
    //     else if (angle == 1) {
    //         // move diagonally (back right)
    //         frontLeftPower = 0;
    //         backLeftPower = -1;
    //         frontRightPower = -1;
    //         backRightPower = 0;
    //     }
    //     else if (angle == 2) {
    //         // move backwards
    //         frontLeftPower = -1;
    //         backLeftPower = -1;
    //         frontRightPower = -1;
    //         backRightPower = -1;
    //     }
    //     else if (angle == 3) {
    //         // move diagonally (back left)
    //         frontLeftPower = -1;
    //         backLeftPower = 0;
    //         frontRightPower = 0;
    //         backRightPower = -1;
    //     }
    //     else if (angle == 4) {
    //         // move to the left
    //         frontLeftPower = -1;
    //         backLeftPower = 1;
    //         frontRightPower = 1;
    //         backRightPower = -1;
    //     }
    //     else if (angle == 5) {
    //         // move diagonally (front left)
    //         frontLeftPower = 0;
    //         backLeftPower = 1;
    //         frontRightPower = 1;
    //         backRightPower = 0;
    //     }
    //     else if (angle == 6) {
    //         // move forwards
    //         frontLeftPower = 1;
    //         backLeftPower = 1;
    //         frontRightPower = 1;
    //         backRightPower = 1;
    //     }
    //     else if (angle == 7) {
    //         // move diagonally (front right)
    //         frontLeftPower = 1;
    //         backLeftPower = 0;
    //         frontRightPower = 0;
    //         backRightPower = 1;
    //     }

    // }

    // frontLeftPower *= magnitude;
    // backLeftPower *= magnitude;
    // frontRightPower *= magnitude;
    // backRightPower *= magnitude;

    // frontLeftPower *= sign(frontLeftPower); //* remap(std::abs(frontLeftPower), 0, 1, 0.5, 1) 
    // backLeftPower *= sign(backLeftPower); //* remap(std::abs(backLeftPower), 0, 1, 0.5, 1)
    // frontRightPower *= sign(frontRightPower); //* remap(std::abs(frontRightPower), 0, 1, 0.5, 1)
    // backRightPower *= sign(backRightPower); //* remap(std::abs(backRightPower), 0, 1, 0.5, 1)

    // set wheel velocities
    // fl_wheel_joint_.setCommand(frontLeftPower);
    // fr_wheel_joint_.setCommand(frontRightPower);
    // bl_wheel_joint_.setCommand(backLeftPower);
    // br_wheel_joint_.setCommand(backRightPower);



}

private:

void cmdVelCallback(const geometry_msgs::Twist &command)
{
    ROS_INFO_STREAM("call cmd-vel cb");
	command_.writeFromNonRT(command);
}

}; // class definition

} // namespace mecanum_drive_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerBase)
