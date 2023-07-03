#ifndef _kangaroo_hpp
#define _kangaroo_hpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdint.h>
#include <boost/thread.hpp>
#include <string>
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

class kangaroo : public rclcpp::Node 
{
public:
    // create 1x 'kangaroo' node 
    kangaroo();
	// ~kangaroo();
	bool open();
	void close();
	bool start();
	void stop();

    void JointStateCB();

	std::string joint_state_topic;
	std::string joint_traj_topic;

private:
    bool is_open() const;
	// Define subscriber callback function:
	void JointTrajCB(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

    // functions for sending information to the kangaroo
    bool send_get_request(unsigned char address, char channel, unsigned char desired_parameter);
	bool set_channel_speed(double speed, unsigned char address, char channel);
	bool send_start_signals(uint8_t address);

    // functions used for the request - response  (JointStateCB)
	int get_kangaroo_parameter(unsigned char address, char channel, unsigned char desired_parameter);
	int read_message(unsigned char address, bool& ok);
	uint8_t read_one_byte(bool& ok);
	int evaluate_kangaroo_response( uint8_t address, uint8_t* header, uint8_t* data, bool& ok);
	void handle_errors(uint8_t address, int error_code);

    // address of the serial port
	std::string port;
	// the number of lines of the encoder
	int encoder_lines_per_revolution;
	// the hertz that the JointState will be published at
	int hz;
	// the joints names for the two motors
	std::string ch1_joint_name;
	std::string ch2_joint_name;

    // file descriptor, which is used for accessing serial ports in C.
	//   it's essentially an address to the serial port
	int fd;

	// ROS2 init for timer callback
	rclcpp::TimerBase::SharedPtr poll_timer;

	// Subscriber init
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_sub;
	// Publisher init
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;

    // mutex-es for accessing the serial that the kangaroo is connected on
	// output_mutex *must* be locked first
	boost::mutex output_mutex;
	boost::mutex input_mutex;

    // unit conversion
	double encoder_lines_to_radians( int encoder_lines );
	int radians_to_encoder_lines( double radians );
};


#endif /* _kangaroo_hpp */