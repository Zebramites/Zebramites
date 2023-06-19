// reads in state from all hardware and then writes it all at once
// saves lots of writes and makes sure that all the data is from the same time

#include <ros/ros.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/io_service.hpp>
#include <std_msgs/Float64.h>

double SPEED_LIMIT = 0.7;
double SPEED_INCREMENT = 0.02;

using boost::asio::serial_port;
using namespace::boost::asio;


class AlignToSubstationAction
{
    protected:

        ros::NodeHandle nh_;
        std::string action_name_;

        ros::Subscriber front_left_motor_sub_;
        ros::Subscriber front_right_motor_sub_;
        ros::Subscriber back_left_motor_sub_;
        ros::Subscriber back_right_motor_sub_;

        ros::Subscriber base_rotate_servo_sub_;
        ros::Subscriber base_arm_servo_sub_;
        ros::Subscriber stage_2_arm_servo_sub_;
        ros::Subscriber claw_servo_sub_;

        std::pair<std::string, std::pair<double, double>> front_left_motor_state_ = {"3", {0.0, 0.0}};
        std::pair<std::string, std::pair<double, double>> front_right_motor_state_ = {"2", {0.0, 0.0}}; 
        std::pair<std::string, std::pair<double, double>> back_left_motor_state_ = {"4", {0.0, 0.0}};
        std::pair<std::string, std::pair<double, double>> back_right_motor_state_ = {"1", {0.0, 0.0}};
        
        std::pair<std::string, std::pair<double, double>> base_rotate_servo_state_ = {"5", {-5.0, -5.0}}; 
        std::pair<std::string, std::pair<double, double>> base_arm_servo_state_ = {"6", {90.0, 90.0}};
        std::pair<std::string, std::pair<double, double>> stage_2_arm_servo_state_ = {"7", {45.0, 45.0}};
        std::pair<std::string, std::pair<double, double>> claw_servo_state_ = {"8", {140.0, 140.0}};
        
        //io_service ios;
        serial_port *port;


    public:

        AlignToSubstationAction(std::string name) :
            front_left_motor_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/front_left_drive_controller/command", 1, &AlignToSubstationAction::front_left_motor_callback, this)),
            front_right_motor_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/front_right_drive_controller/command", 1, &AlignToSubstationAction::front_right_motor_callback, this)),
            back_left_motor_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/back_left_drive_controller/command", 1, &AlignToSubstationAction::back_left_motor_callback, this)),
            back_right_motor_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/back_right_drive_controller/command", 1, &AlignToSubstationAction::back_right_motor_callback, this)),
            base_rotate_servo_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/base_rotate_controller/command", 1, &AlignToSubstationAction::base_rotate_servo_callback, this)),
            base_arm_servo_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/base_arm_controller/command", 1, &AlignToSubstationAction::base_arm_servo_callback, this)),
            stage_2_arm_servo_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/stage_two_arm_controller/command", 1, &AlignToSubstationAction::stage_2_arm_servo_callback, this)),
            claw_servo_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/claw_controller/command", 1, &AlignToSubstationAction::claw_servo_callback, this))
        {
            ROS_INFO_STREAM("Port set to " << "/dev/rfcomm0");
            io_service ios;
            port = new serial_port(ios, "/dev/rfcomm0");
            try {
                port->set_option(boost::asio::serial_port_base::baud_rate(115200));
            } catch (boost::system::system_error::exception e) {
                ROS_ERROR_STREAM("error setting serial port baud rate");
                exit(-1);
            }
        }

        ~AlignToSubstationAction(void)
        {

        }

        std::string double_to_two_decimals(double val) {
            std::string cmd_string = std::to_string(val);
            cmd_string = cmd_string.substr(0, cmd_string.find(".") + 3);
            return cmd_string;
        }
        
        std::string cool_rate_limit(std::pair<double, double>& speed_pair) {
            double wanted_speed = speed_pair.first;

            if (fabs(wanted_speed) <= SPEED_LIMIT) {
                speed_pair.second = wanted_speed;
                return double_to_two_decimals(speed_pair.second);
            }


            if (wanted_speed > 0) {
                if (speed_pair.second < SPEED_LIMIT) {
                    speed_pair.second = SPEED_LIMIT;
                    return double_to_two_decimals(speed_pair.second);
                }
                else {
                    speed_pair.second = std::min(1.0, speed_pair.second + SPEED_INCREMENT);
                }
            } 
 
            else { // negative speed
                if (speed_pair.second > -SPEED_LIMIT) {
                    speed_pair.second = -SPEED_LIMIT;
                    return double_to_two_decimals(speed_pair.second);
                }
                else {
                    speed_pair.second = std::max(-1.0, speed_pair.second - SPEED_INCREMENT);
                }
            }
            return double_to_two_decimals(speed_pair.second);
        }
        void write_to_hardware() {
            // message format 
            // z<MOTORID>;<SPEED>;<MOTORID>;<SPEED>;-9;-9 MESSAGE ENDS
            

            std::string message;

            ROS_INFO_STREAM("Front left motor state: " << front_left_motor_state_.first << " " << front_left_motor_state_.second.first << " " << front_left_motor_state_.second.second);
            /* 
            message += "z" + front_left_motor_state_.first + ";" + double_to_two_decimals(front_left_motor_state_.second) + ";";
            message += front_right_motor_state_.first + ";" + double_to_two_decimals(front_right_motor_state_.second) + ";";
            message += back_left_motor_state_.first + ";" + double_to_two_decimals(back_left_motor_state_.second) + ";";
            message += back_right_motor_state_.first + ";" + double_to_two_decimals(back_right_motor_state_.second) + ";";
            message += base_rotate_servo_state_.first + ";" + double_to_two_decimals(base_rotate_servo_state_.second) + ";";
            message += base_arm_servo_state_.first + ";" + double_to_two_decimals(base_arm_servo_state_.second) + ";";
            message += stage_2_arm_servo_state_.first + ";" + double_to_two_decimals(stage_2_arm_servo_state_.second) + ";";
            message += claw_servo_state_.first + ";" + double_to_two_decimals(claw_servo_state_.second) + ";";
            message += "-9;-9";
            message += "\n";
            */
           ///* 
            message += "z" + front_left_motor_state_.first + ";" + cool_rate_limit(front_left_motor_state_.second) + ";";
            message += front_right_motor_state_.first + ";" + cool_rate_limit(front_right_motor_state_.second) + ";";
            message += back_left_motor_state_.first + ";" + cool_rate_limit(back_left_motor_state_.second) + ";";
            message += back_right_motor_state_.first + ";" + cool_rate_limit(back_right_motor_state_.second) + ";";
            message += base_rotate_servo_state_.first + ";" + cool_rate_limit(base_rotate_servo_state_.second) + ";";
            message += base_arm_servo_state_.first + ";" + cool_rate_limit(base_arm_servo_state_.second) + ";";
            message += stage_2_arm_servo_state_.first + ";" + cool_rate_limit(stage_2_arm_servo_state_.second) + ";";
            message += claw_servo_state_.first + ";" + cool_rate_limit(claw_servo_state_.second) + ";";
            message += "-9;-9";
            message += "\n";
            //*/
            ROS_INFO_STREAM("Sending message: " << message);
            auto cs = message.c_str();
            port->write_some(const_buffer(cs, strlen(cs)));
        }

        void main_loop() {
            ros::Rate r(10);
            while (ros::ok()) {
                ros::spinOnce();
                write_to_hardware();
                r.sleep();
            }
        }

        void front_left_motor_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            front_left_motor_state_.first = "3";
            front_left_motor_state_.second.first = msg->data * -1;
        }

        void front_right_motor_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            front_right_motor_state_.first = "2";
            front_right_motor_state_.second.first = msg->data;
        }

        void back_left_motor_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            back_left_motor_state_.first = "4";
            back_left_motor_state_.second.first = msg->data * -1 ;
        }

        void back_right_motor_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            back_right_motor_state_.first = "1";
            back_right_motor_state_.second.first = msg->data;
        }

        void base_rotate_servo_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            base_rotate_servo_state_.first = "5";
            base_rotate_servo_state_.second.first = msg->data;
        }

        void base_arm_servo_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            base_arm_servo_state_.first = "6";
            base_arm_servo_state_.second.first = msg->data;
        }

        void stage_2_arm_servo_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            stage_2_arm_servo_state_.first = "7";
            stage_2_arm_servo_state_.second.first = msg->data;
        }

        void claw_servo_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            claw_servo_state_.first = "8";
            claw_servo_state_.second.first = msg->data;
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "align_to_substation");

    AlignToSubstationAction alignToGrid("align_to_substation");
    alignToGrid.main_loop();
    ros::spin();

    return 0;
}