#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/UInt8.h>
#include <px4_autonomy/Position.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Takeoff.h>
#include "Eigen/Dense"

#define STATE_TAKEOFF 					1
#define STATE_LAND 						2
#define STATE_IMAGE_CTL_AFTER_TAKEOFF 	3
#define STATE_IMAGE_CTL_BEFORE_LAND		4
#define STATE_FLY						5
#define STATE_HOVERING					6
#define STATE_WAITING					7
#define STATE_CROSS						8
#define TARGET_LANDING					9
#define TARGET_CROSS_CIRCLE				10

#define VEL_XY							2.0
#define VEL_Z							1.5

using namespace std;
using namespace Eigen;

float fly_height[9] = { 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
Matrix<float, 4, 9> Reletive_pos;
Reletive_pos<<
	1.6,  -1.5,  0,   TARGET_LANDING,      	//1-2Parking
	0.15,  3.2,  0,   TARGET_CROSS_CIRCLE, 	//2-3Cross
	0.7,  -1.6,  0,   TARGET_LANDING,		//3-4P
	1.85, -1.55, 0,   TARGET_CROSS_CIRCLE,	//4-5C
	2.95,  2.2,  0,   TARGET_CROSS_CIRCLE,	//5-6C
   -2.78,  0.8,  0,   TARGET_LANDING,		//6-7P
	4.83,  0.0,  0,   TARGET_LANDING,		//7-8P
   -0.6,  -2.65, 0,   TARGET_CROSS_CIRCLE,	//8-9C
   -0.6,  -2.65, 0,   TARGET_LANDING;		//9-10P

int vehicle_status = 0;
int px4_status = 0;
int current_num = 0;
int counter = 0;
mavros_msgs::State previous_state;
mavros_msgs::State current_state;

px4_autonomy::Position current_pos;
px4_autonomy::Position pos_stamp;
geometry_msgs::Pose2D image_pos;

bool isArrived_xy(px4_autonomy::Position pos, px4_autonomy::Position pos_sp)
{
	if(sqrt((pos_sp.x - pos.x)*(pos_sp.x - pos.x) + (pos_sp.y - pos.y)*(pos_sp.y - pos.y))< 0.2) 
		return true;
	else
		return false;
}

bool isArrived_z(px4_autonomy::Position pos, px4_autonomy::Position pos_sp)
{
	if(fabs(pos_sp.z - pos.z) < 0.1) 
		return true;
	else
		return false;
}

bool image_control(geometry_msgs::Pose2D pos, px4_autonomy::Velocity vel_sp)
{

}

void state_callback(const mavros_msgs::State::ConstPtr& msg){
	previous_state = current_state;
    current_state = *msg;
}

void px4_status_callback(const std_msgs::UInt8 &msg)
{
	px4_status =  msg.data;
}

void pose_callback(const px4_autonomy::Position &msg)
{
	current_pos = msg;
}

void image_callback(const project_3::CamInfo &msg)
{
	image_pos.x = 
	image_pos.y = 
	image_pos.theta = 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, 'control');
	ros::NodeHandle n;
	ros::Subscriber state_sub = n.subscribe("mavros/state", 1, state_callback);
	ros::Subscriber status_sub = n.subscribe("/px4/status", 1, px4_status_callback);
	ros::Subscriber pose_sub = n.subscribe("/px4/pose", 1, pose_callback );
	ros::Subscriber image_sub = n.subscribe("/camera/pose", 1, image_callback);
	ros::Publisher takeoff_pub = n.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1); 
	ros::Publisher pose_pub = n.advertise<px4_autonomy::Position>("/px4/cmd_pose", 1); 
    ros::Publisher vel_pub = n.advertise<px4_autonomy::Velocity>("/px4/cmd_vel", 1); 
	ros::Rate loop_rate(20);

	while(ros:ok())
	{
		if(current_state.mode != "OFFBOARD")
		{
			continue;
		}else
		{
			if(previous_state != "OFFBOARD")
			{
				vehicle_status = STATE_WAITING;
			}

			switch(vehicle_status)
			{
				case STATE_WAITING:
				{
					if(counter < 50)
					{
						counter ++;
					}else
					{
						counter = 0;
						vehicle_status = STATE_TAKEOFF;
					}
					break;
				}

				case STATE_TAKEOFF:
				{
					if(px4_status != 5)
					{
						px4_autonomy::Takeoff takeOff;
						takeOff.take_off = 1; 
		    			takeoff_pub.publish(takeOff);
					}else
					{
						//fly to set height
						px4_autonomy::Position pos_sp;
						pos_sp.x = current_pos.x;
						pos_sp.y = current_pos.y;
						pos_sp.z = fly_height[current_num];
						if(isArrived_z(current_pos, pos_sp))
						{
							px4_autonomy::Velocity vel_sp; 
							vel_sp.header.stamp = ros::Time::now();
							vel_sp.x = 0.0;
							vel_sp.y = 0.0;
							vel_sp.z = 0.0
							vel_sp.yaw_rate = 0.0;	
							vel_pub.publish(vel_sp);
							//Switch to image control
							vehicle_status = STATE_IMAGE_CTL_AFTER_TAKEOFF;

						}else
						{
							//rise the height
							px4_autonomy::Velocity vel_sp; 
							vel_sp.header.stamp = ros::Time::now();
							vel_sp.x = 0.0;
							vel_sp.y = 0.0;
							vel_sp.z = 0.8;
							vel_sp.yaw_rate = 0.0;	
							vel_pub.publish(vel_sp);
						}
					}
					break;
				}

				case STATE_HOVERING:
				{

					break;
				}

				case STATE_IMAGE_CTL_AFTER_TAKEOFF:
				{
					px4_autonomy::Velocity vel_sp; 
					if(!image_control(image_pos,vel_sp))
					{
						vel_pub.publish(vel_sp);
					}else
					{
						px4_autonomy::Velocity vel_sp; 
						vel_sp.header.stamp = ros::Time::now();
						vel_sp.x = 0.0;
						vel_sp.y = 0.0;
						vel_sp.z = 0.0
						vel_sp.yaw_rate = 0.0;	
						vel_pub.publish(vel_sp);

						//record current position
						pos_stamp = current_pos;
						//Switch to fly 
						vehicle_status = STATE_FLY;

					}

					break;
				}

				case STATE_IMAGE_CTL_BEFORE_LAND:
				{

					
					break;
				}

				case STATE_FLY:
				{
					if(Reletive_pos(current_num, 4) == TARGET_LANDING)
					{	
						//next will be landing 
						px4_autonomy::Position pos_sp;
						pos_sp.x = pos_stamp.x + Reletive_pos(current_num, 0);
						pos_sp.y = pos_stamp.y + Reletive_pos(current_num, 1);
						pos_sp.z = pos_stamp.z;

						if(isArrived_xy(current_pos, pos_sp))
						{
							px4_autonomy::Velocity vel_sp; 
							vel_sp.header.stamp = ros::Time::now();
							vel_sp.x = 0.0;
							vel_sp.y = 0.0;
							vel_sp.z = 0.0;
							vel_sp.yaw_rate = 0.0;	
							vel_pub.publish(vel_sp);

							//Switch to image_control
							vehicle_status = STATE_IMAGE_CTL_BEFORE_LAND;

						}else
						{
							float distance = sqrt((pos_sp.x - pos.x)*(pos_sp.x - pos.x) + (pos_sp.y - pos.y)*(pos_sp.y - pos.y));
							px4_autonomy::Velocity vel_sp; 
							vel_sp.header.stamp = ros::Time::now();
							vel_sp.x = (pos_sp.x - pos.x) / distance * VEL_XY;
							vel_sp.y = (pos_sp.y - pos.y) / distance * VEL_XY;
							vel_sp.z = 0.0;
							vel_sp.yaw_rate = 0.0;	
							vel_pub.publish(vel_sp);
						}

					}else
					{
						//next will be crossing

					}
	

					

					break;
				}

				case STATE_LAND:
				{

					
					break;
				}

				case STATE_CROSS:
				{

					
					break;
				}

					

			}

		}


		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}