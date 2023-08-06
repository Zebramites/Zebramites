// reads in state from all hardware and then writes it all at once
// saves lots of writes and makes sure that all the data is from the same time

#include <ros/ros.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/io_service.hpp>
#include <std_msgs/Float64.h>
#include <boost/asio.hpp>
#include <minimotor_msgs/RPYEuler.h>
#include <minimotor_msgs/ArmFastPass.h>
// ros angles
#include <angles/angles.h>
// add std_srvs 
#include <std_srvs/Empty.h>

double SPEED_LIMIT = 0.7;
double SPEED_INCREMENT = 0.02;

using boost::asio::serial_port;
using namespace::boost::asio;


class HardwareSync
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

        //ros::Subscriber arm_fast_pass_sub_;

        std::pair<std::string, std::pair<double, double>> front_left_motor_state_ = {"3", {0.0, 0.0}};
        std::pair<std::string, std::pair<double, double>> front_right_motor_state_ = {"2", {0.0, 0.0}}; 
        std::pair<std::string, std::pair<double, double>> back_left_motor_state_ = {"4", {0.0, 0.0}};
        std::pair<std::string, std::pair<double, double>> back_right_motor_state_ = {"1", {0.0, 0.0}};
        
        std::pair<std::string, std::pair<double, double>> base_rotate_servo_state_ = {"5", {-5.0, -5.0}}; 
        std::pair<std::string, std::pair<double, double>> base_arm_servo_state_ = {"6", {170.0, 175.0}};
        std::pair<std::string, std::pair<double, double>> stage_2_arm_servo_state_ = {"7", {40.0, 40.0}};
        std::pair<std::string, std::pair<double, double>> claw_servo_state_ = {"8", {60.0, 60.0}};
        
        double imu_yaw_offset = -900;
        double imu_pitch_offset = -900;
        double last_pitch = 0;
        double last_yaw = 0;
        bool send_imu_data_ = false; 
        bool want_to_run_auto_ = false;
        //io_service ios;
        serial_port *bt_serial_port;
        // make publisher for rpy euler
        ros::Publisher rpy_euler_pub = nh_.advertise<minimotor_msgs::RPYEuler>("/minibot/imu_state", 1);
        // make service for zeroing imu
        ros::ServiceServer zero_imu_srv = nh_.advertiseService("/imu/zero_imu", &HardwareSync::rezero_imu, this);
        ros::Timer imu_timer_ = nh_.createTimer(ros::Duration(0.04), boost::bind(&HardwareSync::read_hardware_and_pub, this));
        ros::ServiceServer stop_imu_srv = nh_.advertiseService("/imu/stop_imu_reads", &HardwareSync::stop_imu_reads, this);
        ros::ServiceServer start_imu_srv = nh_.advertiseService("/imu/start_imu_reads", &HardwareSync::start_imu_reads, this);
        ros::ServiceServer start_auto_srv = nh_.advertiseService("/minibot/start_auto", &HardwareSync::start_auto, this);


    public:

        HardwareSync(std::string name) :
            front_left_motor_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/front_left_drive_controller/command", 1, &HardwareSync::front_left_motor_callback, this)),
            front_right_motor_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/front_right_drive_controller/command", 1, &HardwareSync::front_right_motor_callback, this)),
            back_left_motor_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/back_left_drive_controller/command", 1, &HardwareSync::back_left_motor_callback, this)),
            back_right_motor_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/back_right_drive_controller/command", 1, &HardwareSync::back_right_motor_callback, this)),
            base_rotate_servo_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/base_rotate_controller/command", 1, &HardwareSync::base_rotate_servo_callback, this)),
            base_arm_servo_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/base_arm_controller/command", 1, &HardwareSync::base_arm_servo_callback, this)),
            stage_2_arm_servo_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/stage_two_arm_controller/command", 1, &HardwareSync::stage_2_arm_servo_callback, this)),
            claw_servo_sub_(nh_.subscribe<std_msgs::Float64>("/minibot/claw_controller/command", 1, &HardwareSync::claw_servo_callback, this))
            //arm_fast_pass_sub_(nh_.subscribe<minimotor_msgs::ArmFastPass>("/minibot/arm_fast_pass_controller/command", 1, &HardwareSync::arm_fast_pass_callback, this))
        {
            imu_timer_.stop();

            ROS_INFO_STREAM("Port set to " << "/dev/rfcomm0");
            io_service ios;
            bt_serial_port = new serial_port(ios, "/dev/rfcomm0");
            try {
                bt_serial_port->set_option(boost::asio::serial_port_base::baud_rate(115200));
            } catch (boost::system::system_error::exception e) {
                ROS_ERROR_STREAM("error setting serial port baud rate");
                exit(-1);
            }
        }

        ~HardwareSync(void)
        {
            // send all motors to 0
            std::cout << "Destructor called, sending all motors to 0" << std::endl;
            std::string message;
            message += "z" + front_left_motor_state_.first + ";" + "0.0" + ";";
            message += front_right_motor_state_.first + ";" + "0.0" + ";";
            message += back_left_motor_state_.first + ";" + "0.0" + ";";
            message += back_right_motor_state_.first + ";" + "0.0" + ";";
            message += base_rotate_servo_state_.first + ";" + double_to_two_decimals(base_rotate_servo_state_.second.first) + ";";
            message += base_arm_servo_state_.first + ";" + double_to_two_decimals(base_arm_servo_state_.second.first) + ";";
            message += stage_2_arm_servo_state_.first + ";" + double_to_two_decimals(stage_2_arm_servo_state_.second.first) + ";";
            message += claw_servo_state_.first + ";" + double_to_two_decimals(claw_servo_state_.second.first) + ";";
            message += "-9;0";
            message += "\n";
            auto cs = message.c_str();
            bt_serial_port->write_some(const_buffer(cs, strlen(cs)));
        }


        std::string double_to_two_decimals(double val) {
            std::string cmd_string = std::to_string(val);
            cmd_string = cmd_string.substr(0, cmd_string.find(".") + 3);
            return cmd_string;
        }
        
        std::string cool_rate_limit(std::pair<double, double>& speed_pair) {
            //return double_to_two_decimals(speed_pair.first);

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

            //ROS_INFO_STREAM("Front left motor state: " << front_left_motor_state_.first << " " << front_left_motor_state_.second.first << " " << front_left_motor_state_.second.second);
            
            message += "z" + front_left_motor_state_.first + ";" + cool_rate_limit(front_left_motor_state_.second) + ";";
            message += front_right_motor_state_.first + ";" + cool_rate_limit(front_right_motor_state_.second) + ";";
            message += back_left_motor_state_.first + ";" + cool_rate_limit(back_left_motor_state_.second) + ";";
            message += back_right_motor_state_.first + ";" + cool_rate_limit(back_right_motor_state_.second) + ";";
            message += base_rotate_servo_state_.first + ";" + double_to_two_decimals(base_rotate_servo_state_.second.first) + ";";
            message += base_arm_servo_state_.first + ";" + double_to_two_decimals(base_arm_servo_state_.second.first) + ";";
            message += stage_2_arm_servo_state_.first + ";" + double_to_two_decimals(stage_2_arm_servo_state_.second.first) + ";";
            message += claw_servo_state_.first + ";" + double_to_two_decimals(claw_servo_state_.second.first) + ";";
            if (want_to_run_auto_) {
                message += "-9;1";
            }
            else {
                message += "-9;0";
            }
            message += "\n";
            ROS_INFO_STREAM_THROTTLE(5, "Sending message: " << message);
            auto cs = message.c_str();
            bt_serial_port->write_some(const_buffer(cs, strlen(cs)));
            // wait for like 2ms
            //sleep(0.02);
        }
        
        bool start_auto(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
            ROS_WARN_STREAM("======Starting Auto!=====");
            want_to_run_auto_ = true;
            stop_imu_reads(req, res);
            return true;
        }


        bool start_imu_reads(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
            ROS_WARN_STREAM("======NOT Starting IMU reads commented out=====");
            //send_imu_data_ = true;
            //imu_timer_.start();
            return true;
        }

        bool stop_imu_reads(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
            ROS_WARN_STREAM("======Stopping IMU reads=====");
            send_imu_data_ = false;
            imu_timer_.stop();
            return true;
        }

        // make this take a std_srvs::Empty::Request& req, std_srvs::Empty::Response& res
        bool rezero_imu(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
            ROS_INFO_STREAM("Rezeroing IMU, new offset is " << last_yaw << " degrees");
            imu_yaw_offset = last_yaw;
            imu_pitch_offset = last_pitch;
            return true;
        }

        void read_hardware_and_pub() {
            boost::asio::streambuf buffer;
            std::string line;
            //std::cout << "Reading from serial port" << std::endl;
            try {
                boost::asio::read_until(*bt_serial_port, buffer, '\n');
                std::istream input_stream(&buffer);
                std::getline(input_stream, line);

                // Process the received line
                if (!line.empty()) {
                    // Assuming the format is 'a<yaw>;<PITCH>z'
                    if (line[0] == 'a' && line[line.size() - 1] == 'z') {
                        line = line.substr(1, line.size() - 2);  // Remove 'a' and 'z'

                        // Extract yaw and pitch values
                        size_t semicolon_pos = line.find(';');
                        if (semicolon_pos != std::string::npos) {
                            std::string yaw_str = line.substr(0, semicolon_pos);
                            std::string pitch_str = line.substr(semicolon_pos + 1);

                            double yaw = std::stod(yaw_str);
                            double pitch = std::stod(pitch_str);
                            if (imu_yaw_offset == -900) {
                                imu_yaw_offset = yaw;
                                ROS_INFO_STREAM("IMU offset set to " << imu_yaw_offset);
                            }
                            if (imu_pitch_offset == -900) {
                                imu_pitch_offset = pitch;
                                ROS_INFO_STREAM("IMU pitch offset set to " << imu_pitch_offset);
                            }
                            last_yaw = yaw;
                            last_pitch = pitch;
                            yaw -= imu_yaw_offset;
                            pitch -= imu_pitch_offset;
                            minimotor_msgs::RPYEuler rpy_msg;
                            rpy_msg.header.stamp = ros::Time::now();
                            // convert to 0 to 360 with ros angles
                            yaw = angles::to_degrees(angles::normalize_angle_positive(angles::from_degrees(yaw)));
                            rpy_msg.yaw = yaw;
                            pitch = angles::to_degrees(angles::normalize_angle(angles::from_degrees(pitch)));
                            rpy_msg.pitch = pitch;
                            rpy_msg.roll = -10000;
                            rpy_euler_pub.publish(rpy_msg);

                            //std::cout << "Received yaw: " << yaw << ", pitch: " << pitch << std::endl;
                        } else {
                            std::cout << "Invalid line format: " << line << std::endl;
                        }
                    } else {
                        std::cout << "Invalid line: " << line << std::endl;
                    }
                }

                buffer.consume(buffer.size());  // Clear the buffer
            } catch (std::exception& e) {
                std::cout << "Error: " << e.what() << std::endl;
                buffer.consume(buffer.size());  // Clear the buffer
            }
        }

        void main_loop() {

            ros::Rate r(10);
            while (ros::ok()) {
                ros::spinOnce();
                write_to_hardware();
                //read_hardware_and_pub();
                r.sleep();
            }
        }
        /*
        void arm_fast_pass_callback(const minimotor_msgs::ArmFastPass::ConstPtr& msg) {
            // pray for no race conditions? idk if its bad if write to hardware is called while this is running
            base_rotate_servo_state_.second.first = msg->base_arm_rotate;
            base_arm_servo_state_.second.first = msg->base_arm_joint;
            stage_2_arm_servo_state_.second.first = msg->stage_2_arm_joint;
            write_to_hardware();
        }
        */
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

    HardwareSync alignToGrid("align_to_substation");
    alignToGrid.main_loop();
    ros::spin();

    return 0;
}