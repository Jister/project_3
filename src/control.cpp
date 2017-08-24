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
#define STATE_ADJUST_YAW				12
#define STATE_FIND_IMAGE				13
#define TARGET_LANDING					14
#define TARGET_CROSS_CIRCLE				15

#define VEL_XY							0.5
#define VEL_CROSS						0.5
#define VEL_UP							1.0
#define VEL_DOWN						-0.8
#define VEL_Z 							0.8
#define VEL_YAW							0.5
#define LAND_HEIGHT						0.6
#define FIND_HEIGHT						2.0
#define IMG_CRITICAL					30 
#define POS_NEAR						0.2
#define IMG_POS_NEAR					0.1
#define Z_NEAR							0.12
#define YAW_NEAR 						0.2
#define IMG_CTL_P 						0.8
#define CROSS_DISTANCE 					1.5
#define STEREO_ARRIVED					0.08
#define YAW_CHANGE_NUM					5
#define YAW_CROSS_NUM					2	
#define FINISHED_NUM					7

#define PI 								3.14

using namespace std;
using namespace Eigen;

float fly_height[9] = { 2.0, 1.1, 1.0, 2.0, 2.0, 1.3, 2.0, 1.5, 1.5};
float cross_height[4] = {2.3, 2.1, 2.5, 1.8};
bool change_yaw = false;
float yaw_sp = PI/2.0;
Matrix<float, 9, 4> Reletive_pos;
float dt = 0.05;

int vehicle_status = 0;
int vehicle_next_status = 0;
int px4_status = 0;
int current_num = 0;
int cross_num = 0;
int counter = 0;
mavros_msgs::State previous_state;
mavros_msgs::State current_state;

bool _reset_pos_sp_xy = false;
bool _reset_pos_sp_z = false;
bool _reset_img_sp = false;
px4_autonomy::Position pos_sp_dt; 
px4_autonomy::Position img_pos_sp;

bool z_arrived = false;
bool takeoff_ready = false;
px4_autonomy::Position current_pos;
px4_autonomy::Position pos_stamp;
bool image_down_valid = false;
bool image_stereo_valid = false;
bool num_valid = false;
geometry_msgs::Pose2D image_pos;
geometry_msgs::Point stereo_pos;
int imageCenter[2];

int image_lost_cnt = 0;

void reset_pos_sp_xy()
{
	if(_reset_pos_sp_xy)
	{
		pos_sp_dt.header.stamp = ros::Time::now();
		pos_sp_dt.x = current_pos.x;
		pos_sp_dt.y = current_pos.y;
		ROS_INFO("Reset pos xy: %f  %f",pos_sp_dt.x, pos_sp_dt.y);
		_reset_pos_sp_xy = false;
	}
}

void reset_pos_sp_z()
{
	if(_reset_pos_sp_z)
	{
		pos_sp_dt.header.stamp = ros::Time::now();
		pos_sp_dt.z = current_pos.z;
		ROS_INFO("Reset pos z: %f",pos_sp_dt.z);
		_reset_pos_sp_z = false;
	}
}

bool isArrived_xy(px4_autonomy::Position &pos, px4_autonomy::Position &pos_sp)
{
	if(sqrt((pos_sp.x - pos.x)*(pos_sp.x - pos.x) + (pos_sp.y - pos.y)*(pos_sp.y - pos.y))< POS_NEAR) 
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


bool isArrived_xy_stereo(px4_autonomy::Position &pos, px4_autonomy::Position &pos_sp)
{
	if(sqrt((pos_sp.x - pos.x)*(pos_sp.x - pos.x) + (pos_sp.y - pos.y)*(pos_sp.y - pos.y))< POS_NEAR) 
	{
		return true;
	}
	else
	{
		if(sqrt((pos_sp.x - pos.x)*(pos_sp.x - pos.x) + (pos_sp.y - pos.y)*(pos_sp.y - pos.y))< 0.5)
		{
			if(image_stereo_valid)
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


bool isArrived_xy_for_img(px4_autonomy::Position &pos, px4_autonomy::Position &pos_sp)
{
	if(sqrt((pos_sp.x - pos.x)*(pos_sp.x - pos.x) + (pos_sp.y - pos.y)*(pos_sp.y - pos.y))< IMG_POS_NEAR) 
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool isArrived_z(px4_autonomy::Position &pos, px4_autonomy::Position &pos_sp)
{
	if(fabs(pos_sp.z - pos.z) < Z_NEAR) 
		return true;
	else
		return false;
}

bool isArrived_yaw(float &current_yaw, float &yaw_sp_1)
{
	if(fabs(yaw_sp_1 - current_yaw) < YAW_NEAR) 
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

// bool image_control(geometry_msgs::Pose2D &pos, px4_autonomy::Velocity &vel_sp)
// {
// 	float P_pos = 0.008;
// 	Vector2f vel_sp_body;
// 	Vector2f vel_sp_world;
// 	Vector2f image_center;
// 	Vector2f image_pos;

// 	image_center(0) = imageCenter[0];
// 	image_center(1) = imageCenter[1];
// 	image_pos(0) = pos.x;
// 	image_pos(1) = pos.y;

// 	ROS_INFO("Image pos: %f  %f",image_pos(0),image_pos(1));
// 	ROS_INFO("Image center: %f  %f",image_center(0),image_center(1));
// 	Vector2f err = image_pos - image_center;
// 	float dist = err.norm();
// 	if(dist < 25)
// 	{
// 		vel_sp.header.stamp = ros::Time::now();
// 		vel_sp.x = 0.0;
// 		vel_sp.y = 0.0;
// 		vel_sp.z = 0.0;
// 		vel_sp.yaw_rate = 0.0;
// 		return true;
// 	}else
// 	{
// 		vel_sp_world = P_pos * err;
// 		//rotate(current_pos.yaw, vel_sp_body, vel_sp_world);

// 		vel_sp.header.stamp = ros::Time::now();
// 		vel_sp.x = vel_sp_world(0);
// 		vel_sp.y = - vel_sp_world(1);
// 		vel_sp.z = 0.0;
// 		vel_sp.yaw_rate = 0.0;
// 		ROS_INFO("VEL_SP: %f  %f",vel_sp.x, vel_sp.y);
// //		return false;
// 		return true;
// 	}
// }

bool isImageReady(geometry_msgs::Pose2D &pos)
{
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
	if(dist < IMG_CRITICAL)
	{
		ROS_INFO("image arrived");
		return true;
	}else
	{
		return false;
	}
}

bool image_control(geometry_msgs::Pose2D &pos, px4_autonomy::Position &pos_sp)
{
	float P_pos_x = IMG_CTL_P * current_pos.z / 296.73;
	float P_pos_y = IMG_CTL_P * current_pos.z / 299.46;
	Vector2f pos_sp_increase;
	Vector2f image_center;
	Vector2f image_pos;

	image_center(0) = imageCenter[0];
	image_center(1) = imageCenter[1];
	image_pos(0) = pos.x;
	image_pos(1) = pos.y;

	// ROS_INFO("Image pos: %f  %f",image_pos(0),image_pos(1));
	// ROS_INFO("Image center: %f  %f",image_center(0),image_center(1));
	Vector2f err_body = image_pos - image_center;
	Vector2f err_world; 

	rotate(PI/2.0 - current_pos.yaw, err_body, err_world);
	float dist = err_body.norm();
	if(dist < IMG_CRITICAL)
	{
		pos_sp.header.stamp = ros::Time::now();
		pos_sp.x = current_pos.x;
		pos_sp.y = current_pos.y;
		pos_sp.z = fly_height[current_num];
		pos_sp.yaw = yaw_sp;

		return true;
	}else
	{
		pos_sp_increase(0) = P_pos_x * err_world(0);
		pos_sp_increase(1) = P_pos_y * err_world(1);
		//rotate(current_pos.yaw, vel_sp_body, vel_sp_world);

		pos_sp.header.stamp = ros::Time::now();
		pos_sp.x = current_pos.x + pos_sp_increase(0);
		pos_sp.y = current_pos.y - pos_sp_increase(1);
		pos_sp.z = fly_height[current_num];
		pos_sp.yaw = yaw_sp;
		ROS_INFO("POS: %f  %f",current_pos.x, current_pos.y);
		ROS_INFO("POS_SP: %f  %f",pos_sp.x, pos_sp.y);
		return false;
	}
}

bool image_fly(px4_autonomy::Position &pos_sp)
{
	reset_pos_sp_xy();
	float distance = sqrt((pos_sp.x - pos_sp_dt.x)*(pos_sp.x - pos_sp_dt.x) + (pos_sp.y - pos_sp_dt.y)*(pos_sp.y - pos_sp_dt.y));
	float vec_x = (pos_sp.x - pos_sp_dt.x) / distance;
	float vec_y = (pos_sp.y - pos_sp_dt.y) / distance;

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
	pos_sp_dt.z = pos_sp.z;
	pos_sp_dt.yaw = yaw_sp;	

	if(isArrived_xy_for_img(current_pos, pos_sp))
	{
		ROS_INFO("update image sp");
		return true;
	}else
	{
		return false;
	}

}

bool isStereoReady()
{
	if(fabs(stereo_pos.y - CROSS_DISTANCE) < STEREO_ARRIVED && fabs(stereo_pos.x) < STEREO_ARRIVED)
	{
		ROS_INFO("STEREO: %f  %f", stereo_pos.y, stereo_pos.x);
		ROS_INFO("stereo ready.");
		Vector2f err;
		err(0) = stereo_pos.x ;
		err(1) = stereo_pos.y - CROSS_DISTANCE;
		Vector2f err_world; 
		rotate(PI/2.0 - current_pos.yaw, err, err_world);
		pos_stamp.x = current_pos.x + err_world(0);
		pos_stamp.y = current_pos.y + err_world(1);
		return true;
	}else
	{
		ROS_INFO("STEREO: %f  %f", stereo_pos.y, stereo_pos.x);
		return false;
	}
}

bool image_control_2(geometry_msgs::Point &pos, px4_autonomy::Position &pos_sp)
{
	Vector2f pos_sp_increase;
	Vector2f err;
	err(0) = pos.x;
	err(1) = pos.y - CROSS_DISTANCE;
	Vector2f err_world; 
	rotate(PI/2.0 - current_pos.yaw, err, err_world);

	float dist = err.norm();
	if(fabs(err(0)) < STEREO_ARRIVED && fabs(err(1)) < STEREO_ARRIVED)
	{
		pos_sp.header.stamp = ros::Time::now();
		pos_sp.x = current_pos.x;
		pos_sp.y = current_pos.y;
		pos_sp.z = fly_height[current_num];
		pos_sp.yaw = yaw_sp;
		return true;
	}else
	{
		pos_sp_increase = 0.6 * err_world;
		//rotate(current_pos.yaw, vel_sp_body, vel_sp_world);

		pos_sp.header.stamp = ros::Time::now();
		pos_sp.x = current_pos.x + pos_sp_increase(0);
		pos_sp.y = current_pos.y + pos_sp_increase(1);
		pos_sp.z = fly_height[current_num];
		pos_sp.yaw = yaw_sp;
		ROS_INFO("POS: %f  %f",current_pos.x, current_pos.y);
		ROS_INFO("POS_SP: %f  %f",pos_sp.x, pos_sp.y);
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
	num_valid = msg.theta_valid;
	image_pos.x = msg.x;
	image_pos.y = msg.y;
	image_pos.theta = msg.theta;
	imageCenter[0] = msg.center_x;
	imageCenter[1] = msg.center_y;
}

void stereo_image_callback(const project_3::Image_info &msg)
{
	image_stereo_valid = msg.valid;
	stereo_pos.y = msg.x;
	stereo_pos.x = -msg.y;
	stereo_pos.z = msg.z;
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

	// Reletive_pos<<
 //   -2.5,  0.0,  0,   TARGET_LANDING,      	//1-2Parking
	// 0.0,  3.0,  0,   TARGET_CROSS_CIRCLE, 	//2-3Cross
	// 0.0,  2.5,  0,   TARGET_LANDING,		//3-4P
 //   -2.75,  0.0,  0,   TARGET_CROSS_CIRCLE,	//4-5C
 //   -0.7, -1.98,  0,   TARGET_CROSS_CIRCLE,	//5-6C
 //   -2.78,  0.8,  0,   TARGET_LANDING,		//6-7P
	// 4.83,  0.0,  0,   TARGET_LANDING,		//7-8P
 //   -0.6,  -2.65, 0,   TARGET_CROSS_CIRCLE,	//8-9C
 //   -0.6,  -2.65, 0,   TARGET_LANDING;		//9-10P

   	Reletive_pos<<
   -2.5,  1.9,  0,   TARGET_LANDING,      	//1-2Parking
   -1.3,  2.0,  0,   TARGET_CROSS_CIRCLE, 	//2-3Cross
	2.0,  2.2,  0,   TARGET_CROSS_CIRCLE,		//3-4P
    3.1,  2.1,  0,   TARGET_LANDING,	//4-5C
    1.8, -3.3,  0,   TARGET_LANDING,	//5-6C
   -1.3, -3.0,  0,   TARGET_CROSS_CIRCLE,		//6-7P
   -1.9, -2.7,  0,   TARGET_LANDING,		//7-8P
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
						if(px4_status == 1)
						{
							counter ++;
						}
					}else
					{
						if(current_num == FINISHED_NUM)
						{
							ROS_INFO("Finished.");
						}else
						{
							counter = 0;
							vehicle_status = STATE_TAKEOFF;
							ROS_INFO("Takeoff");
						}
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
		    			_reset_pos_sp_xy = true;
		    			_reset_pos_sp_z = true;
					}else
					{
						//fly to set height
						px4_autonomy::Position pos_sp;
						pos_sp.x = current_pos.x;
						pos_sp.y = current_pos.y;
						pos_sp.z = fly_height[current_num];
						if(isArrived_z(current_pos, pos_sp))
						{
							_reset_pos_sp_xy = true;
							reset_pos_sp_xy();
							pos_sp_dt.z = pos_sp.z;
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);

							if(current_num == YAW_CHANGE_NUM)
							{
								vehicle_status = STATE_HOVERING;
								vehicle_next_status = STATE_ADJUST_YAW;
								_reset_pos_sp_xy = true; 
							}else
							{
								//Switch to image control
								vehicle_status = STATE_HOVERING;
								vehicle_next_status = STATE_IMAGE_CTL_AFTER_TAKEOFF;
								_reset_pos_sp_xy = true;
								_reset_img_sp = true;
							}
							
							ROS_INFO("Takeoff ready...");

						}else
						{
							ROS_INFO("TAKEOFF RISING");
							//rise the height
							reset_pos_sp_xy();
							reset_pos_sp_z();
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
							pos_sp_dt.yaw = yaw_sp;	
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
						reset_pos_sp_xy();
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.yaw = yaw_sp;	
						pose_pub.publish(pos_sp_dt);
					}else
					{
						counter = 0;
						vehicle_status = vehicle_next_status;
					}
					break;
				}

				case STATE_IMAGE_CTL_AFTER_TAKEOFF:
				{
					ROS_INFO("Image control after takeoff.");
					if(image_down_valid)
					{
						if(isImageReady(image_pos) && num_valid)
						{
							_reset_pos_sp_xy = true;
							reset_pos_sp_xy();

							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.z = fly_height[current_num];
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);

							//record current position
							pos_stamp = current_pos;
							//reset pos_sp
							_reset_pos_sp_xy = true;
							//Switch to fly 
							vehicle_next_status = STATE_FLY;
							vehicle_status = STATE_HOVERING;
							ROS_INFO("Image Control Ready");
						}else
						{
							ROS_INFO("Image control");
							//set _reset_pos_sp_xy true before
							reset_pos_sp_xy();
							if(_reset_img_sp)
							{
								img_pos_sp.x = current_pos.x;
								img_pos_sp.y = current_pos.y;
								img_pos_sp.z = fly_height[current_num];
								_reset_img_sp = false;
							}
							if(image_fly(img_pos_sp))	//get image sp, update new img sp
							{
								image_control(image_pos, img_pos_sp);
								pose_pub.publish(pos_sp_dt);
							}else
							{
								pos_sp_dt.header.stamp = ros::Time::now();
								ROS_INFO("sp:          %f   %f", pos_sp_dt.x, pos_sp_dt.y);
								ROS_INFO("current pos: %f   %f", current_pos.x, current_pos.y);
								ROS_INFO("%d", px4_status);
								pose_pub.publish(pos_sp_dt);
							}
						}
					}else
					{
						ROS_INFO("image not valid");
						image_lost_cnt ++;
						if(image_lost_cnt > 40)
						{
							image_lost_cnt = 0;
							vehicle_next_status = vehicle_status;
							vehicle_status = STATE_FIND_IMAGE;
						}
					}

					break;
				}

				case STATE_IMAGE_CTL_BEFORE_LAND:
				{
					ROS_INFO("Image control before land.");
					if(image_down_valid)
					{
						if(isImageReady(image_pos) && num_valid)
						{
							_reset_pos_sp_xy = true;
							reset_pos_sp_xy();

							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.z = fly_height[current_num];
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);

							//record current position
							pos_stamp = current_pos;
							//reset pos_sp
							_reset_pos_sp_xy = true;
							_reset_pos_sp_z = true;
							//Switch to fly 
							vehicle_next_status = STATE_LAND;
							vehicle_status = STATE_HOVERING;
							ROS_INFO("Image Control Ready Before Landing");
						}else
						{
							ROS_INFO("Image control Before Landing");
							//set _reset_pos_sp_xy true before
							reset_pos_sp_xy();
							if(_reset_img_sp)
							{
								img_pos_sp.x = current_pos.x;
								img_pos_sp.y = current_pos.y;
								img_pos_sp.z = fly_height[current_num];
								_reset_img_sp = false;
							}
							if(image_fly(img_pos_sp))	//get image sp, update new img sp
							{
								image_control(image_pos, img_pos_sp);
								pose_pub.publish(pos_sp_dt);
							}else
							{
								pos_sp_dt.header.stamp = ros::Time::now();
								ROS_INFO("sp:          %f   %f", pos_sp_dt.x, pos_sp_dt.y);
								ROS_INFO("current pos: %f   %f", current_pos.x, current_pos.y);
								ROS_INFO("%d", px4_status);
								pose_pub.publish(pos_sp_dt);
							}
						}
					}else
					{
						ROS_INFO("image not valid");
						image_lost_cnt ++;
						if(image_lost_cnt > 40)
						{
							image_lost_cnt = 0;
							vehicle_next_status = vehicle_status;
							vehicle_status = STATE_FIND_IMAGE;
						}
					}

					break;
				}

				case STATE_FLY:
				{
					ROS_INFO("Flying...");
					//reset current setpoint
					reset_pos_sp_xy();
					if(Reletive_pos(current_num, 3) == TARGET_LANDING)
					{	
						//next will be landing 
						ROS_INFO("Next will be landing...");
						px4_autonomy::Position pos_sp;
						pos_sp.x = pos_stamp.x + Reletive_pos(current_num, 0);
						pos_sp.y = pos_stamp.y + Reletive_pos(current_num, 1);
						pos_sp.z = fly_height[current_num];

						if(isArrived_xy(current_pos, pos_sp))
						{
							_reset_pos_sp_xy = true;
							reset_pos_sp_xy();

							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.z = pos_sp.z;
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);

							//Switch to image_control
							_reset_pos_sp_xy = true;
							_reset_img_sp = true;
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
							pos_sp_dt.z = pos_sp.z;
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);
						}

					}else
					{
						//next will be crossing
						ROS_INFO("Next will be crossing...");
						px4_autonomy::Position pos_sp;	
						if(cross_num == YAW_CROSS_NUM && current_num == YAW_CHANGE_NUM)
						{
							pos_sp.x = pos_stamp.x + Reletive_pos(current_num, 0);
							pos_sp.y = pos_stamp.y + Reletive_pos(current_num, 1) + CROSS_DISTANCE;
						}else
						{
							pos_sp.x = pos_stamp.x + Reletive_pos(current_num, 0);
							pos_sp.y = pos_stamp.y + Reletive_pos(current_num, 1) - CROSS_DISTANCE;
						}
						
						pos_sp.z = fly_height[current_num];

						if(isArrived_xy_stereo(current_pos, pos_sp))
						{
							_reset_pos_sp_xy = true;
							reset_pos_sp_xy();

							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.z = pos_sp.z;
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);

							//Switch to image_control before cross
							_reset_pos_sp_xy = true;
							_reset_img_sp = true;
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
							pos_sp_dt.z = pos_sp.z;
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);
						}
					}
					break;
				}

				case STATE_LAND:
				{
					px4_autonomy::Position pos_sp;
					pos_sp.x = pos_stamp.x;
					pos_sp.y = pos_stamp.y;
					pos_sp.z = LAND_HEIGHT;

					if(isArrived_z(current_pos, pos_sp))
					{
						ROS_INFO("Landing...");
						_reset_pos_sp_xy = true;
						_reset_pos_sp_z = true;
						reset_pos_sp_xy();
						reset_pos_sp_z();
						vehicle_status = STATE_WAITING;
						current_num ++;
						px4_autonomy::Takeoff takeOff;
						takeOff.take_off = 2; 
		    			takeoff_pub.publish(takeOff);

					}else
					{
						ROS_INFO("Adjust height before land");
						//adjust the height
						reset_pos_sp_z();
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
						pos_sp_dt.yaw = yaw_sp;	
						pose_pub.publish(pos_sp_dt);
					}
					
					break;
				}

				case STATE_IMAGE_CTL_BEFORE_CROSS:
				{
					ROS_INFO("Image control before cross.");
					if(image_stereo_valid)
					{
						if(isStereoReady())
						{
							_reset_pos_sp_xy = true;
							reset_pos_sp_xy();

							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.z = fly_height[current_num];
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);

							//record current position
							//pos_stamp = current_pos;
							//move it to isStereoReady
							//reset pos_sp
							_reset_pos_sp_xy = true;
							_reset_pos_sp_z = true;
							//Switch to fly 
							vehicle_next_status = STATE_ADJUST_Z_BEFORE_CROSS;
							vehicle_status = STATE_HOVERING;
							ROS_INFO("Image Control Ready Before Crossing");
						}else
						{
							ROS_INFO("Image control Before crossing");
							//set _reset_pos_sp_xy true before
							reset_pos_sp_xy();
							if(_reset_img_sp)
							{
								img_pos_sp.x = current_pos.x;
								img_pos_sp.y = current_pos.y;
								img_pos_sp.z = fly_height[current_num];
								_reset_img_sp = false;
							}
							if(image_fly(img_pos_sp))	//get image sp, update new img sp
							{
								image_control_2(stereo_pos, img_pos_sp);
								pose_pub.publish(pos_sp_dt);
							}else
							{
								pos_sp_dt.header.stamp = ros::Time::now();
								ROS_INFO("sp:          %f   %f", pos_sp_dt.x, pos_sp_dt.y);
								ROS_INFO("current pos: %f   %f", current_pos.x, current_pos.y);
								ROS_INFO("%d", px4_status);
								pose_pub.publish(pos_sp_dt);
							}
						}
					}else
					{
						ROS_INFO("image not valid");
					}

					break;
				}

				case STATE_CROSS:
				{
					ROS_INFO("Crossing...");
					reset_pos_sp_xy();

					px4_autonomy::Position pos_sp;
					if(cross_num == YAW_CROSS_NUM && current_num == YAW_CHANGE_NUM)
					{
						pos_sp.x = pos_stamp.x;
						pos_sp.y = pos_stamp.y - 2.0 * CROSS_DISTANCE;
					}else
					{
						pos_sp.x = pos_stamp.x;
						pos_sp.y = pos_stamp.y + 2.0 * CROSS_DISTANCE;
					}
					
					pos_sp.z = cross_height[cross_num];

					if(isArrived_xy(current_pos, pos_sp))
					{
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.x = pos_sp.x;
						pos_sp_dt.y = pos_sp.y;
						pos_sp_dt.z = pos_sp.z;
						pos_sp_dt.yaw = yaw_sp;	
						pose_pub.publish(pos_sp_dt);

						//Switch to flying 
						vehicle_status = STATE_HOVERING;
						vehicle_next_status = STATE_ADJUST_Z_AFTER_CROSS;
						
						_reset_pos_sp_z = true;
						pos_stamp.x = pos_stamp.x;
						if(cross_num == YAW_CROSS_NUM && current_num == YAW_CHANGE_NUM)
						{
							pos_stamp.y = pos_stamp.y - CROSS_DISTANCE;
						}else
						{
							pos_stamp.y = pos_stamp.y + CROSS_DISTANCE;
						}
						
						pos_stamp.z = current_pos.z; 
						current_num++;
						cross_num++;

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
							pos_sp_dt.x += vec_x * VEL_CROSS * dt;
						}
						
						if(fabs(pos_sp.y - pos_sp_dt.y) < 0.01)
						{
							pos_sp_dt.y = pos_sp.y;
						}else
						{
							pos_sp_dt.y += vec_y * VEL_CROSS * dt;
						}
						pos_sp_dt.z = pos_sp.z;
						pos_sp_dt.yaw = yaw_sp;	
						pose_pub.publish(pos_sp_dt);
					}				
					break;
				}	
				case STATE_ADJUST_Z_BEFORE_CROSS:
				{
					ROS_INFO("Adjust Z before cross");
					//fly to set height
					px4_autonomy::Position pos_sp;
					pos_sp.x = pos_stamp.x;
					pos_sp.y = pos_stamp.y;
					pos_sp.z = cross_height[cross_num];

					if(isArrived_z(current_pos, pos_sp))
					{
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.x = pos_sp.x;
						pos_sp_dt.y = pos_sp.y;
						pos_sp_dt.z = pos_sp.z; 
						pos_sp_dt.yaw = yaw_sp;	
						pose_pub.publish(pos_sp_dt);
						//Switch to image control
						vehicle_status = STATE_HOVERING;
						vehicle_next_status = STATE_CROSS;
						ROS_INFO("Adjust Z ready...");

					}else
					{
						//rise the height
						reset_pos_sp_xy();
						reset_pos_sp_z();
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
						pos_sp_dt.yaw = yaw_sp;	
						pose_pub.publish(pos_sp_dt);
					}

					break;
				}
				case STATE_ADJUST_Z_AFTER_CROSS:
				{
					ROS_INFO("Adjust Z after cross.");
					//fly to set height
					px4_autonomy::Position pos_sp;
					pos_sp.x = current_pos.x;
					pos_sp.y = current_pos.y;
					pos_sp.z = fly_height[current_num];

					if(isArrived_z(current_pos, pos_sp))
					{
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.yaw = yaw_sp;	
						pose_pub.publish(pos_sp_dt);
						vehicle_status = STATE_HOVERING;
						vehicle_next_status = STATE_FLY;
						_reset_pos_sp_xy = true;
						ROS_INFO("Adjust Z ready...");
					}else
					{
						//adjust the height
						reset_pos_sp_z();
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
						pos_sp_dt.yaw = yaw_sp;	
						pose_pub.publish(pos_sp_dt);
					}

					break;
				}

				case STATE_ADJUST_YAW:
				{
					yaw_sp = -PI/2.0;
					ROS_INFO("Adjust Yaw.");
					//fly to set height
					px4_autonomy::Position pos_sp;
					pos_sp.x = current_pos.x;
					pos_sp.y = current_pos.y;
					pos_sp.z = fly_height[current_num];
					pos_sp.yaw = yaw_sp;

					if(isArrived_yaw(current_pos.yaw, pos_sp.yaw))
					{
						ROS_INFO("Yaw ready.");
						_reset_pos_sp_xy = true;
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.z = pos_sp.z;
						pos_sp_dt.yaw = yaw_sp;
						pose_pub.publish(pos_sp_dt);

						_reset_pos_sp_xy= true;
						_reset_img_sp = true;
						vehicle_status = STATE_HOVERING;
						vehicle_next_status = STATE_IMAGE_CTL_AFTER_TAKEOFF;

					}else
					{
						reset_pos_sp_xy();
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.z = pos_sp.z;

						if(fabs(pos_sp.yaw - pos_sp_dt.yaw) < 0.01)
						{
							pos_sp_dt.yaw = pos_sp.yaw;
						}else
						{
							pos_sp_dt.yaw +=  -VEL_YAW * dt;
						}
						pose_pub.publish(pos_sp_dt);
					}

					break;
				}

				case STATE_FIND_IMAGE:
				{
					if(image_down_valid)
					{
						ROS_INFO("FInd down image!!!");
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.x = current_pos.x;
						pos_sp_dt.y = current_pos.y;
						pos_sp_dt.z = current_pos.z; 
						pos_sp_dt.yaw = yaw_sp;	
						pose_pub.publish(pos_sp_dt);
						vehicle_status = STATE_HOVERING;

					}else
					{
						ROS_INFO("Looking for image...");
						//fly to set height
						px4_autonomy::Position pos_sp;
						pos_sp.x = current_pos.x;
						pos_sp.y = current_pos.y;
						pos_sp.z = FIND_HEIGHT;

						if(isArrived_z(current_pos, pos_sp))
						{
							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.x = pos_sp.x;
							pos_sp_dt.y = pos_sp.y;
							pos_sp_dt.z = pos_sp.z; 
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);
							//Switch to image control
							vehicle_status = STATE_HOVERING;
							ROS_INFO("Find image Finished...");

						}else
						{
							//rise the height
							reset_pos_sp_xy();
							reset_pos_sp_z();
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
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);
						}
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
