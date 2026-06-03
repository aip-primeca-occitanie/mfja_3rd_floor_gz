//controller header
// Command manipulates only the protobuf messages, these messages are then serialized as string in main .cpp and send thanks to sockets properties.
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "socket.hpp"

#include <string>
#include <netinet/in.h> // sockaddr_in
#include <sys/socket.h> // socket, bind, recvfrom
#include <arpa/inet.h>  // inet_addr
#include <unistd.h>     // close
#include <vector>

//librairies protobuf
#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <protos/api_v2_rtos.pb.h>

//#define DEBUG 1
//#ifdef DEBUG
// code here
//#endif

#define PERIOD (1.0/250.0)

struct RobotConfig {
 	public :
 	std::string brand = "Staubli";
 	std::string model = "TX2-60L";
 	std::string controller_model = "CS9";
 	//std::string address = "192.168.10.254"; Mis en commentaire en suivant l'exemple n°2 reçu de Fuzzy
 	// values to define the robot's cell limitation
 	// float x_axis_user_min
 	// float x_axis_user_max
 	// float y_axis_user_min
 	// float y_axis_user_max
 	// float z_axis_user_min
 	// float z_axis_user_max
 	float joint_min_position = 0;// valeur par défaut présente dans le code de Fuzzy Logic
 	float joint_max_position = 1000;// valeur par défaut présente dans le code de Fuzzy Logic
 	float max_velocity = 1000;// valeur par défaut présente dans le code de Fuzzy Logic
 	float max_acceleration = 5000;// valeur par défaut présente dans le code de Fuzzy Logic
 	float max_jerk = 10000;// valeur par défaut présente dans le code de Fuzzy Logic
};

enum class ControlMode {
		jogging = 1,
		cartesian = 2,
	};

enum class Direction {

	//none = 0,
	x = 1,
	y = 2,
	z = 3,
};

class Command {
	public :
	Command();//constructor : Automatically called when an object is created

	bool init_driver(Socket& socket);
	bool reset_driver(Socket& socket);
	int control_mode(Socket& socket, ControlMode mode); //jog mode dans exemple Fuzzy mais on pourrait imaginer d'autres modes
	std::vector<double> get_joint_position(Socket& socket);
	std::vector<double> get_tool_position(Socket& socket);
	void send_joint_position(int joint_index, double delta);
	bool send_tool_position(Direction direction, Socket& socket,double delta);
	void print_joint_position(Socket& socket);
	void print_tool_position(Socket& socket);
	void print_system_state(Socket& socket);

	private :
	int command_fd_;
	RobotConfig robot_;
	ControlMode selected_mode_ = ControlMode::jogging;//Default value is jogging, it could be changed in the future if we want to set up other modes
	//Direction direction_ = Direction::none; 
	flr_api::v2::rtos::SystemState receive_system_state(Socket& socket);
	void send_cartesian_pose(flr_api::v2::rtos::CartesianPoseJoggingCmd& jog_cmd_msg,Socket& socket);
	void compute_jacobian(std::vector<double> joint_position);
	void compute_error();
	bool is_initialized_driver_=false;
	bool is_reset_driver_=false;

};

#endif