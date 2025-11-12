#include "ros/ros.h"
#include <termios.h>

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"

int num_actuators = 0;
int control_rate = 0;
int num_modules = 0;

char getch(){
    fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0)
		ROS_INFO("no_key_pressed");
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}

int main(int argc, char **argv){

    ros::init(argc, argv,"keyboard_node");
    ros::NodeHandle n;
    
    // Declare publishers 
    ros::Publisher key_pub = n.advertise<std_msgs::Int32MultiArray>("origami_vs/velocity",1);
    ros::Publisher mode_pub = n.advertise<std_msgs::Int32>("origami_vs/OMMD_control_mode",1);

    // Import requried parameters
    n.getParam("origami_skeleton_vs/control_rate", control_rate);
    n.getParam("origami_skeleton_vs/no_of_actuators", num_actuators);
    n.getParam("origami_skeleton_vs/no_of_modules", num_modules);
    
    // Choosing velocity control mode
    std_msgs::Int32 mode_msg;
    mode_msg.data = 2;
    
    // loop rate
    ros::Rate loop_rate(control_rate);
    
    while(ros::ok()){

        // Keyboard listener
        int c = getch();
        
        std_msgs::Int32MultiArray vel_msg;    //Velocity message
        vel_msg.data.resize(num_actuators);   // resize msg
        
        mode_pub.publish(mode_msg);         // publish control mode
        
        // Expand module 1
        if (c == 'w'){
            ROS_INFO("%c", c);
            vel_msg.data[0] = 100;
            vel_msg.data[1] = 100;
            vel_msg.data[2] = 0;
            vel_msg.data[3] = 0;
            key_pub.publish(vel_msg);
        }

        // Expand module 2
        if (c == 'W'){
            ROS_INFO("%c", c);
            vel_msg.data[0] = 0;
            vel_msg.data[1] = 0;
            vel_msg.data[2] = 100;
            vel_msg.data[3] = 100;
            key_pub.publish(vel_msg);
        }

        // Retract module 1
        else if (c == 's'){
            ROS_INFO("%c", c);
            vel_msg.data[0] = -100;
            vel_msg.data[1] = -100;
            vel_msg.data[2] = 0;
            vel_msg.data[3] = 0;
            key_pub.publish(vel_msg);
        }

        // Retract module 2
        else if (c == 'S'){
            ROS_INFO("%c", c);
            vel_msg.data[0] = 0;
            vel_msg.data[1] = 0;
            vel_msg.data[2] = -100;
            vel_msg.data[3] = -100;
            key_pub.publish(vel_msg);
        }
        // Bend module 1 left
        else if (c == 'a'){
            ROS_INFO("%c", c);
            vel_msg.data[0] = 50;
            vel_msg.data[1] = 100;
            vel_msg.data[2] = 0;
            vel_msg.data[3] = 0;
            key_pub.publish(vel_msg);
        }

        // Bend module 2 left
        else if (c == 'A'){
            ROS_INFO("%c", c);
            vel_msg.data[0] = 0;
            vel_msg.data[1] = 0;
            vel_msg.data[2] = 50;
            vel_msg.data[3] = 100;
            key_pub.publish(vel_msg);
        }

        // Bend module 1 right 
        else if (c == 'd'){
            ROS_INFO("%c", c);
            vel_msg.data[0] = 100;
            vel_msg.data[1] = 50;
            vel_msg.data[2] = 0;
            vel_msg.data[3] = 0;
            key_pub.publish(vel_msg);
        }

        // Bend module 2 right
        else if (c == 'D'){
            ROS_INFO("%c", c);
            vel_msg.data[0] = 0;
            vel_msg.data[1] = 0;
            vel_msg.data[2] = 100;
            vel_msg.data[3] = 50;
            key_pub.publish(vel_msg);
        }

        else if(c == 'p' || c == 'P'){
            vel_msg.data[0] = 0.0;
            vel_msg.data[1] = 0.0;
            vel_msg.data[2] = 0.0;
            vel_msg.data[3] = 0.0;
            key_pub.publish(vel_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}