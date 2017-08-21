#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8.h>
#include <px4_autonomy/Position.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Takeoff.h>
#include <mavros_msgs/State.h>
#include "project_3/Image_info.h"
#include "Eigen/Dense"

#define STATE_TAKEOFF 					1
#define STATE_LAND 						2
#define STATE_IMAGE_CTL_AFTER_TAKEOFF 	3
#define STATE_IMAGE_CTL_BEFORE_LAND		4
#define STATE_FLY						5
#define STATE_HOVERING					6
#define STATE_WAITING					7
#define STATE_IMAGE_CTL_BEFORE_CROSS	8
#define STATE_CROSS						9
#define STATE_ADJUST_Z_BEFORE_CROSS		10
#define STATE_ADJUST_Z_AFTER_CROSS		11
#define TARGET_LANDING					12
#define TARGET_CROSS_CIRCLE				13

#define VEL_XY							0.8
#define VEL_UP							1.0
#define VEL_DOWN						-0.8
#define VEL_Z 							0.8
#define LAND_HEIGHT						1.0

#define PI 								3.14

using namespace std;
using namespace Eigen;

float fly_height[9] = { 1.8, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
float cross_height[4] = {1.2, 1.4, 1.6, 1.8};
Matrix<float, 9, 4> Reletive_pos;
// Reletive_pos.resize(9,4);

int vehicle_status = 0;
int vehicle_next_status = 0;
int px4_status = 0;
int current_num = 0;
int cross_num = 0;
int counter = 0;
mavros_msgs::State previous_state;
mavros_msgs::State current_state;

bool _reset_pos_sp = false;
px4_autonomy::Position pos_sp_dt; 

bool z_arrived = false;
bool takeoff_ready = false;
px4_autonomy::Position current_pos;
px4_autonomy::Position pos_stamp;
bool image_down_valid = false;
geometry_msgs::Pose2D image_pos;
geometry_msgs::Point stereo_pos;
int imageCenter[2];

void reset_pos_sp()
{
	if(_reset_pos_sp)
	{
		pos_sp_dt.x = current_pos.x;
		pos_sp_dt.y = current_pos.y;
		pos_sp_dt.z = current_pos.z;
		ROS_INFO("Reset pos: %f  %f  %f",pos_sp_dt.x, pos_sp_dt.y, pos_sp_dt.z);
		_reset_pos_sp = false;
	}
}

bool isArrived_xy(px4_autonomy::Position &pos, px4_autonomy::Position &pos_sp)
{
	if(sqrt((pos_sp.x - pos.x)*(pos_sp.x - pos.x) + (pos_sp.y - pos.y)*(pos_sp.y - pos.y))< 0.2) 
	{
		return true;
	}
	else
	{
		if(sqrt((pos_sp.x - pos.x)*(pos_sp.x - pos.x) + (pos_sp.y - pos.y)*(pos_sp.y - pos.y))< 0.5)
		{
			if(image_down_valid)
			{
				return true;
			}else
			{
				return false;
			}
		}else
		{
			return false;
		}
	}
}

bool isArrived_z(px4_autonomy::Position &pos, px4_autonomy::Position &pos_sp)
{
	if(fabs(pos_sp.z - pos.z) < 0.1) 
		return true;
	else
		return false;
}

void rotate(float theta,  const Vector2f& input,  Vector2f& output)
{
	float sy = sinf(theta);
	float cy = cosf(theta);

	Matrix2f data;
	data(0,0) = cy;
	data(0,1) = -sy;
	data(1,0) = sy;
	data(1,1) = cy;

	output = data * input;
}

bool image_control(geometry_msgs::Pose2D &pos, px4_autonomy::Velocity &vel_sp)
{
	float P_pos = 0.008;
	Vector2f vel_sp_body;
	Vector2f vel_sp_world;
	Vector2f image_center;
	Vector2f image_pos;

	image_center(0) = imageCenter[0];
	image_center(1) = imageCenter[1];
	image_pos(0) = pos.x;
	image_pos(1) = pos.y;

	ROS_INFO("Image pos: %f  %f",image_pos(0),image_pos(1));
	ROS_INFO("Image center: %f  %f",image_center(0),image_center(1));
	Vector2f err = image_pos - image_center;
	float dist = err.norm();
	if(dist < 25)
	{
		vel_sp.header.stamp = ros::Time::now();
		vel_sp.x = 0.0;
		vel_sp.y = 0.0;
		vel_sp.z = 0.0;
		vel_sp.yaw_rate = 0.0;

		return true;
	}else
	{
		vel_sp_world = P_pos * err;
		//rotate(current_pos.yaw, vel_sp_body, vel_sp_world);

		vel_sp.header.stamp = ros::Time::now();
		vel_sp.x = vel_sp_world(0);
		vel_sp.y = - vel_sp_world(1);
		vel_sp.z = 0.0;
		vel_sp.yaw_rate = 0.0;
		ROS_INFO("VEL_SP: %f  %f",vel_sp.x, vel_sp.y);
//		return false;
		return true;
	}
}

bool image_control_2(geometry_msgs::Point &pos, px4_autonomy::Velocity &vel_sp)
{
	float P_pos = 0.5;
	Vector2f image_center;
	Vector2f image_pos;
	float image_pos_z;
	Vector2f vel_sp_world;

	image_pos(0) = pos.x;
	image_pos(1) = pos.y;
	image_pos_z = pos.z;

	image_center(0) = 0.0;
	image_center(1) = 1.0;

	Vector2f err = image_pos - image_center;
	float err_z = image_pos_z;

	if(err(0) < 0.1)
	{
		if(err(1) < 0.1)
		{
			vel_sp.header.stamp = ros::Time::now();
			vel_sp.x = 0.0;
			vel_sp.y = 0.0;
			vel_sp.z = 0.0;
			vel_sp.yaw_rate = 0.0;

			return true;
		}else
		{
			vel_sp_world = P_pos * err;
			vel_sp.header.stamp = ros::Time::now();
			vel_sp.x = vel_sp_world(0);
			vel_sp.y = 0.0;
			vel_sp.z = 0.0;
			vel_sp.yaw_rate = 0.0;

			return false;
		}
	}else
	{
		vel_sp_world = P_pos * err;
		vel_sp.header.stamp = ros::Time::now();
		vel_sp.x = 0.0;
		vel_sp.y = vel_sp_world(1);
		vel_sp.z = 0.0;
		vel_sp.yaw_rate = 0.0;

		return false;
	}

}

void state_callback(const mavros_msgs::State &msg){
	previous_state = current_state;
    current_state = msg;
}

void px4_status_callback(const std_msgs::UInt8 &msg)
{
	px4_status =  msg.data;
}

void pose_callback(const px4_autonomy::Position &msg)
{
	current_pos = msg;
}

void image_callback(const project_3::Image_info &msg)
{
	image_down_valid = msg.valid;
	image_pos.x = msg.x;
	image_pos.y = msg.y;
	image_pos.theta = msg.theta;
	imageCenter[0] = msg.center_x;
	imageCenter[1] = msg.center_y;
}

void stereo_image_callback(const geometry_msgs::PointStamped &msg)
{
	stereo_pos.x = msg.point.x;
	stereo_pos.y = msg.point.z;
	stereo_pos.z = msg.point.y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle n;
	ros::Subscriber state_sub = n.subscribe("/mavros/state", 1, state_callback);
	ros::Subscriber status_sub = n.subscribe("/px4/status", 1, px4_status_callback);
	ros::Subscriber pose_sub = n.subscribe("/px4/pose", 1, pose_callback );
	ros::Subscriber image_sub = n.subscribe("/camera/pose", 1, image_callback);
	ros::Subscriber image_stereo_sub = n.subscribe("/stereo/pose", 1, stereo_image_callback);
	ros::Publisher takeoff_pub = n.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1); 
	ros::Publisher pose_pub = n.advertise<px4_autonomy::Position>("/px4/cmd_pose", 1); 
    ros::Publisher vel_pub = n.advertise<px4_autonomy::Velocity>("/px4/cmd_vel", 1); 
	ros::Rate loop_rate(20);

	float dt = 0.05;

	Reletive_pos<<
   -2.65,  0.0,  0,   TARGET_LANDING,      	//1-2Parking
	2.65,  0.0,  0,   TARGET_CROSS_CIRCLE, 	//2-3Cross
	0.7,  -1.6,  0,   TARGET_LANDING,		//3-4P
	1.85, -1.55, 0,   TARGET_CROSS_CIRCLE,	//4-5C
	2.95,  2.2,  0,   TARGET_CROSS_CIRCLE,	//5-6C
   -2.78,  0.8,  0,   TARGET_LANDING,		//6-7P
	4.83,  0.0,  0,   TARGET_LANDING,		//7-8P
   -0.6,  -2.65, 0,   TARGET_CROSS_CIRCLE,	//8-9C
   -0.6,  -2.65, 0,   TARGET_LANDING;		//9-10P

	while(ros::ok())
	{
		if(current_state.mode != "OFFBOARD")
		{
			loop_rate.sleep();
			ros::spinOnce();
			continue;
		}else
		{

			if(previous_state.mode != "OFFBOARD")
			{
				vehicle_status = STATE_WAITING;
				ROS_INFO("OFFBOARD");
			}

			switch(vehicle_status)
			{
				case STATE_WAITING:
				{
					ROS_INFO("Waiting......");
					if(counter < 50)
					{
						counter ++;
					}else
					{
						counter = 0;
						vehicle_status = STATE_TAKEOFF;
						ROS_INFO("Takeoff");
					}
					break;
				}

				case STATE_TAKEOFF:
				{
					if(px4_status == 1 || px4_status == 2)
					{
						px4_autonomy::Takeoff takeOff;
						takeOff.take_off = 1; 
		    			takeoff_pub.publish(takeOff);
		    			_reset_pos_sp = true;
					}else
					{
						//fly to set height
						px4_autonomy::Position pos_sp;
						pos_sp.x = current_pos.x;
						pos_sp.y = current_pos.y;
						pos_sp.z = fly_height[current_num];
						if(isArrived_z(current_pos, pos_sp))
						{
							_reset_pos_sp = true;
							reset_pos_sp();
							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.yaw = PI / 2;	
							pose_pub.publish(pos_sp_dt);
							//Switch to image control
							vehicle_status = STATE_HOVERING;
							vehicle_next_status = STATE_IMAGE_CTL_AFTER_TAKEOFF;
							ROS_INFO("Takeoff ready...");

						}else
						{
							ROS_INFO("TAKEOFF RISING");
							//rise the height
							reset_pos_sp();
							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.x = pos_sp.x;
							pos_sp_dt.y = pos_sp.y;
							if(pos_sp_dt.z < pos_sp.z)
							{
								pos_sp_dt.z += VEL_UP * dt;
							}else
							{
								pos_sp_dt.z = pos_sp.z;
							}
							pos_sp_dt.yaw = PI / 2.0;	
							ROS_INFO("%f", pos_sp_dt.z);
							pose_pub.publish(pos_sp_dt);
						}
					}
					break;
				}

				case STATE_HOVERING:
				{
					ROS_INFO("Hovering......");
					if(counter < 20)
					{
						counter ++;

						px4_autonomy::Velocity vel_sp; 
						vel_sp.header.stamp = ros::Time::now();
						vel_sp.x = 0.0;
						vel_sp.y = 0.0;
						vel_sp.z = 0.0;
						vel_sp.yaw_rate = 0.0;	
						vel_pub.publish(vel_sp);
					}else
					{
						counter = 0;
						vehicle_status = vehicle_next_status;
					}
					break;
				}

				case STATE_IMAGE_CTL_AFTER_TAKEOFF:
				{
					px4_autonomy::Velocity vel_sp; 
					if(!image_control(image_pos,vel_sp))
					{
						vel_pub.publish(vel_sp);
						ROS_INFO("Image control");
					}else
					{
						px4_autonomy::Velocity vel_sp; 
						vel_sp.header.stamp = ros::Time::now();
						vel_sp.x = 0.0;
						vel_sp.y = 0.0;
						vel_sp.z = 0.0;
						vel_sp.yaw_rate = 0.0;	
						vel_pub.publish(vel_sp);

						//record current position
						pos_stamp = current_pos;
						//reset pos_sp
						_reset_pos_sp = true;
						//Switch to fly 
						vehicle_next_status = STATE_FLY;
						vehicle_status = STATE_HOVERING;
						ROS_INFO("Image Control Ready");
					}

					break;
				}

				case STATE_IMAGE_CTL_BEFORE_LAND:
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
						vel_sp.z = 0.0;
						vel_sp.yaw_rate = 0.0;	
						vel_pub.publish(vel_sp);

						//record current position
						pos_stamp = current_pos;
						//Switch to land
						vehicle_status = STATE_HOVERING;
						vehicle_next_status = STATE_LAND;
						ROS_INFO("Image Control Ready");
					}	
					break;
				}

				case STATE_FLY:
				{
					ROS_INFO("Flying...");
					reset_pos_sp();
					if(Reletive_pos(current_num, 3) == TARGET_LANDING)
					{	
						//next will be landing 
						ROS_INFO("Next will be landing...");
						px4_autonomy::Position pos_sp;
						pos_sp.x = pos_stamp.x + Reletive_pos(current_num, 0);
						pos_sp.y = pos_stamp.y + Reletive_pos(current_num, 1);
						pos_sp.z = pos_stamp.z;

						if(isArrived_xy(current_pos, pos_sp))
						{
							_reset_pos_sp = true;
							reset_pos_sp();

							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.yaw = PI / 2;	
							pose_pub.publish(pos_sp_dt);

							//Switch to image_control
							vehicle_status = STATE_HOVERING;
							vehicle_next_status = STATE_IMAGE_CTL_BEFORE_LAND;

						}else
						{
							float distance = sqrt((pos_sp.x - pos_sp_dt.x)*(pos_sp.x - pos_sp_dt.x) + (pos_sp.y - pos_sp_dt.y)*(pos_sp.y - pos_sp_dt.y));
							float vec_x = (pos_sp.x - pos_sp_dt.x) / distance;
							float vec_y = (pos_sp.y - pos_sp_dt.y) / distance;

							pos_sp_dt.header.stamp = ros::Time::now();
							if(fabs(pos_sp.x - pos_sp_dt.x) < 0.01)
							{
								pos_sp_dt.x = pos_sp.x;
							}else
							{
								pos_sp_dt.x += vec_x * VEL_XY * dt;
							}
							
							if(fabs(pos_sp.y - pos_sp_dt.y) < 0.01)
							{
								pos_sp_dt.y = pos_sp.y;
							}else
							{
								pos_sp_dt.y += vec_y * VEL_XY * dt;
							}
							pos_sp_dt.z = pos_sp.y;
							pos_sp_dt.yaw = PI/2.0;	
							pose_pub.publish(pos_sp_dt);
						}

					}else
					{
						//next will be crossing
						ROS_INFO("Next will be crossing...");
						reset_pos_sp();
						px4_autonomy::Position pos_sp;
						
						pos_sp.x = pos_stamp.x + Reletive_pos(current_num, 0);
						pos_sp.y = pos_stamp.y + Reletive_pos(current_num, 1) - 1.0;
						pos_sp.z = pos_stamp.z;

						if(isArrived_xy(current_pos, pos_sp))
						{
							_reset_pos_sp = true;
							reset_pos_sp();

							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.yaw = PI / 2;	
							pose_pub.publish(pos_sp_dt);

							//Switch to image_control before cross
							vehicle_status = STATE_HOVERING;
							vehicle_next_status = STATE_IMAGE_CTL_BEFORE_CROSS;
						}else
						{
							float distance = sqrt((pos_sp.x - pos_sp_dt.x)*(pos_sp.x - pos_sp_dt.x) + (pos_sp.y - pos_sp_dt.y)*(pos_sp.y - pos_sp_dt.y));
							float vec_x = (pos_sp.x - pos_sp_dt.x) / distance;
							float vec_y = (pos_sp.y - pos_sp_dt.y) / distance;

							pos_sp_dt.header.stamp = ros::Time::now();
							if(fabs(pos_sp.x - pos_sp_dt.x) < 0.01)
							{
								pos_sp_dt.x = pos_sp.x;
							}else
							{
								pos_sp_dt.x += vec_x * VEL_XY * dt;
							}
							
							if(fabs(pos_sp.y - pos_sp_dt.y) < 0.01)
							{
								pos_sp_dt.y = pos_sp.y;
							}else
							{
								pos_sp_dt.y += vec_y * VEL_XY * dt;
							}
							pos_sp_dt.z = pos_sp.y;
							pos_sp_dt.yaw = PI/2.0;	
							pose_pub.publish(pos_sp_dt);
						}
					}
					break;
				}

				case STATE_LAND:
				{
					px4_autonomy::Position pos_sp;
					pos_sp.x = current_pos.x;
					pos_sp.y = current_pos.y;
					pos_sp.z = LAND_HEIGHT;

					px4_autonomy::Velocity vel_sp; 
					if(image_control(image_pos,vel_sp) && isArrived_z(current_pos, pos_sp))
					{
						vehicle_status = STATE_WAITING;
						current_num ++;
						px4_autonomy::Takeoff takeOff;
						takeOff.take_off = 2; 
		    			takeoff_pub.publish(takeOff);
		    			ROS_INFO("LANDING......");
					}else
					{
						vel_sp.z = (pos_sp.z - current_pos.z) * 1.0;
						if(vel_sp.z > VEL_UP) vel_sp.z = VEL_UP;
						if(vel_sp.z < VEL_DOWN) vel_sp.z = VEL_DOWN;
						vel_pub.publish(vel_sp);
					}
					
					break;
				}

				case STATE_IMAGE_CTL_BEFORE_CROSS:
				{
					//if(!image_control_2())
					if(0)
					{

					}else
					{
						vehicle_next_status = STATE_ADJUST_Z_BEFORE_CROSS;
						vehicle_status = STATE_HOVERING;
						_reset_pos_sp = true;
						pos_stamp = current_pos;

					}

					break;
				}

				case STATE_CROSS:
				{
					ROS_INFO("Crossing...");
					reset_pos_sp();

					px4_autonomy::Position pos_sp;
					pos_sp.x = pos_stamp.x;
					pos_sp.y = pos_stamp.y + 2.0;
					pos_sp.z = cross_height[cross_num];

					if(isArrived_xy(current_pos, pos_sp))
					{
						_reset_pos_sp = true;
						reset_pos_sp();

						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.yaw = PI / 2;	
						pose_pub.publish(pos_sp_dt);

						//Switch to flying 
						vehicle_status = STATE_HOVERING;
						vehicle_next_status = STATE_ADJUST_Z_AFTER_CROSS;
						current_num++;
						cross_num++;
						_reset_pos_sp = true;
						pos_stamp.x = current_pos.x;
						pos_stamp.y = current_pos.y - 1.0;
						pos_stamp.z = current_pos.z; 

					}else
					{
						float distance = sqrt((pos_sp.x - pos_sp_dt.x)*(pos_sp.x - pos_sp_dt.x) + (pos_sp.y - pos_sp_dt.y)*(pos_sp.y - pos_sp_dt.y));
						float vec_x = (pos_sp.x - pos_sp_dt.x) / distance;
						float vec_y = (pos_sp.y - pos_sp_dt.y) / distance;

						pos_sp_dt.header.stamp = ros::Time::now();
						if(fabs(pos_sp.x - pos_sp_dt.x) < 0.01)
						{
							pos_sp_dt.x = pos_sp.x;
						}else
						{
							pos_sp_dt.x += vec_x * VEL_XY * dt;
						}
						
						if(fabs(pos_sp.y - pos_sp_dt.y) < 0.01)
						{
							pos_sp_dt.y = pos_sp.y;
						}else
						{
							pos_sp_dt.y += vec_y * VEL_XY * dt;
						}
						pos_sp_dt.z = pos_sp.y;
						pos_sp_dt.yaw = PI/2.0;	
						pose_pub.publish(pos_sp_dt);
					}				
					break;
				}	
				case STATE_ADJUST_Z_BEFORE_CROSS:
				{
					//fly to set height
					px4_autonomy::Position pos_sp;
					pos_sp.x = pos_stamp.x;
					pos_sp.y = pos_stamp.y;
					pos_sp.z = cross_height[current_num];

					if(isArrived_z(current_pos, pos_sp))
					{
						_reset_pos_sp = true;
						reset_pos_sp();
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.yaw = PI / 2;	
						pose_pub.publish(pos_sp_dt);
						//Switch to image control
						vehicle_status = STATE_HOVERING;
						vehicle_next_status = STATE_CROSS;
						_reset_pos_sp = true;
						ROS_INFO("Adjust Z ready...");

					}else
					{
						//rise the height
						reset_pos_sp();
						
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.x = pos_sp.x;
						pos_sp_dt.y = pos_sp.y;
						if(pos_sp_dt.z < pos_sp.z)
						{
							pos_sp_dt.z += VEL_UP * dt;
						}else
						{
							pos_sp_dt.z = pos_sp.z;
						}
						pos_sp_dt.yaw = PI / 2.0;	
						pose_pub.publish(pos_sp_dt);
					}

					break;
				}
				case STATE_ADJUST_Z_AFTER_CROSS:
				{
					//fly to set height
					px4_autonomy::Position pos_sp;
					pos_sp.x = pos_stamp.x;
					pos_sp.y = pos_stamp.y;
					pos_sp.z = fly_height[current_num];

					if(isArrived_z(current_pos, pos_sp))
					{
						_reset_pos_sp = true;
						reset_pos_sp();
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.yaw = PI / 2;	
						pose_pub.publish(pos_sp_dt);
						//Switch to image control
						vehicle_status = STATE_HOVERING;
						vehicle_next_status = STATE_FLY;
						_reset_pos_sp = true;
						pos_stamp.z = current_pos.z;
						ROS_INFO("Adjust Z ready...");

					}else
					{
						//adjust the height
						reset_pos_sp();
						

						float vec_z = (pos_sp.z - pos_sp_dt.z) / fabs(pos_sp.z - pos_sp_dt.z);

						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.x = pos_sp.x;
						pos_sp_dt.y = pos_sp.y;
						if(fabs(pos_sp.z - pos_sp_dt.z) < 0.01)
						{
							pos_sp_dt.z = pos_sp.z;
						}else
						{
							pos_sp_dt.z += vec_z * VEL_Z * dt;
						}
						pos_sp_dt.yaw = PI / 2.0;	
						pose_pub.publish(pos_sp_dt);
					}

					break;
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}