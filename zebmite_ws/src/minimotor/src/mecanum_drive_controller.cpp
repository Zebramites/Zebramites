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
    
    double new_y = -curr_cmd.linear.y;
    double new_x = curr_cmd.linear.x;
    double rx = -curr_cmd.angular.z;
    
    double denominator = std::max(std::abs(new_y) + std::abs(new_x) + std::abs(rx), 1.0d);
    double frontLeftPower = (new_y + new_x + rx) / denominator;
    double backLeftPower = (new_y - new_x + rx) / denominator;
    double frontRightPower = (new_y - new_x - rx) / denominator;
    double backRightPower = (new_y + new_x - rx) / denominator;

    // frontLeftPower *= sign(frontLeftPower); //* remap(std::abs(frontLeftPower), 0, 1, 0.5, 1) 
    // backLeftPower *= sign(backLeftPower); //* remap(std::abs(backLeftPower), 0, 1, 0.5, 1)
    // frontRightPower *= sign(frontRightPower); //* remap(std::abs(frontRightPower), 0, 1, 0.5, 1)
    // backRightPower *= sign(backRightPower); //* remap(std::abs(backRightPower), 0, 1, 0.5, 1)

    // set wheel velocities
    fl_wheel_joint_.setCommand(frontLeftPower);
    fr_wheel_joint_.setCommand(frontRightPower);
    bl_wheel_joint_.setCommand(backLeftPower);
    br_wheel_joint_.setCommand(backRightPower);
}

private:

void cmdVelCallback(const geometry_msgs::Twist &command)
{
	command_.writeFromNonRT(command);
}

}; // class definition

} // namespace mecanum_drive_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerBase)
