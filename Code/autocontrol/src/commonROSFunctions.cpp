#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sys/socket.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
 #include <fcntl.h> 
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

std::vector<std::string> split(std::string str,std::string sep)
{
    char* cstr=const_cast<char*>(str.c_str());
    char* current;
    std::vector<std::string> arr;
    current=strtok(cstr,sep.c_str());
    while(current!=NULL){
	arr.push_back(current);
	current=strtok(NULL,sep.c_str());
    }
    return arr;
}

//Returns the Odometry information based on the input data
nav_msgs::Odometry setOdometry_Information(nav_msgs::Odometry odom,float* odom_data)
	{
		odom.pose.pose.position.x= odom_data[0]; 
		odom.pose.pose.position.y= odom_data[1]; 
		odom.pose.pose.position.z= odom_data[2];
		odom.pose.pose.orientation.x=odom_data[3];
		odom.pose.pose.orientation.y=odom_data[4];
		odom.pose.pose.orientation.z=odom_data[5];
		odom.pose.pose.orientation.w=odom_data[6];
		odom.twist.twist.linear.x=odom_data[7];
		odom.twist.twist.linear.y=odom_data[8];
		odom.twist.twist.linear.z=odom_data[9];						
		odom.twist.twist.angular.x=odom_data[10];
		odom.twist.twist.angular.y=odom_data[11];
		odom.twist.twist.angular.z=odom_data[12];
		for (int i=0; i<36; i++) 
		{
			odom.twist.covariance[i]=0;
			odom.pose.covariance[i]=0;
		}

		return odom;
	}

//Returns the Odometry information based on the input angle values, Looks for the world transform to a local frame and set the odometry based on that information.
nav_msgs::Odometry rotate_vehicle(float rad_value_x,float rad_value_y,float angle_value_z,nav_msgs::Odometry odom,float x_val,float y_val,float z_val,std::string frame_to_watch)
	{	
		double Roll,Pitch,Yaw;
		tf::Quaternion q;
		tf::StampedTransform Pose_trans;
		tf::TransformListener listener11;
		tf::Transform transform_new;
		tf::TransformBroadcaster br;

		transform_new.setOrigin(tf::Vector3(0.0,0.0,0.0));
	
		if (angle_value_z == 0.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,0.0));}
		else if (angle_value_z == 90.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,1.5708));}
		else if (angle_value_z == -90.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,-1.5708));}
		else if (angle_value_z == 180.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,3.14159));}
		else if (angle_value_z == -45.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,-0.785398));} 
		else if (angle_value_z == 45.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,0.785398));} 
		else if (angle_value_z == 135.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,2.35619));}
		else if (angle_value_z == 225.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,3.92699));}
		else if (angle_value_z == 10.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,0.174533));}
		else if (angle_value_z == 20.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,0.349066));}
		else if (angle_value_z == 30.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,0.523599));}
		else if (angle_value_z == 40.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,0.698132));}
		else if (angle_value_z == 50.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,0.872665));}
		else if (angle_value_z == 60.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,1.0472));}
		else if (angle_value_z == 70.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,1.22173));}
		else if (angle_value_z == 80.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,1.39626));}
		else if (angle_value_z == 90.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,1.5708));}
		else if (angle_value_z == 100.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,1.74533));}
		else if (angle_value_z == 110.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,1.91986));}
		else if (angle_value_z == 120.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,2.0944));}
		else if (angle_value_z == 130.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,2.26893));}
		else if (angle_value_z == 140.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,2.44346));}
		else if (angle_value_z == 150.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,2.61799));}
		else if (angle_value_z == 160.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,2.79253));}
		else if (angle_value_z == 170.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,2.96706));}
		else if (angle_value_z == 180.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,3.14159));}
		else if (angle_value_z == 190.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,3.31613));}
		else if (angle_value_z == 200.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,3.49066));}
		else if (angle_value_z == 210.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,3.66519));}
		else if (angle_value_z == 220.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,3.83972));}
		else if (angle_value_z == 230.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,4.01426));}
		else if (angle_value_z == 240.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,4.18879));}
		else if (angle_value_z == 250.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,4.36332));}
		else if (angle_value_z == 260.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,4.53786));}
		else if (angle_value_z == 270.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,4.71239));}
		else if (angle_value_z == 280.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,4.88692));}
		else if (angle_value_z == 290.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,5.06145));}
		else if (angle_value_z == 300.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,5.23599));}
		else if (angle_value_z == 310.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,5.41052));}
		else if (angle_value_z == 320.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,5.58505));}
		else if (angle_value_z == 330.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,5.75959));}
		else if (angle_value_z == 340.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,5.93412));}
		else if (angle_value_z == 350.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,6.10865));}
		else if (angle_value_z == 360.0) {transform_new.setRotation(tf::createQuaternionFromRPY(rad_value_x,rad_value_y,6.28319));}

		br.sendTransform(tf::StampedTransform(transform_new, ros::Time::now(),"world",frame_to_watch));
	
		try
		{
			listener11.waitForTransform("world",frame_to_watch,ros::Time(0),ros::Duration(10.0));
			listener11.lookupTransform("world",frame_to_watch,ros::Time(0),Pose_trans);
		}

		catch (tf::TransformException &ex) 
		{
			ROS_ERROR("%s",ex.what());
			//ros::Duration(1.0).sleep();
		}

		Pose_trans.getBasis().getRPY(Roll,Pitch,Yaw);
		q = tf::createQuaternionFromRPY(Roll,Pitch,Yaw);


		float data_list[13]={x_val,y_val,z_val,q[0],q[1],q[2],q[3],0.0,0.0,0.0,0.0,0.0,0.0};
		odom = setOdometry_Information(odom,data_list);

		return odom;
	}

//Returns the odometry based on the x,y,z,angular rate x, angular rate y and angular rate z)
nav_msgs::Odometry apply_movement(std::vector<float> Vehicle_velocity,nav_msgs::Odometry odom)
	{
		float vel_x,vel_y,vel_z,rot_x,rot_y,rot_z;
		vel_x=Vehicle_velocity[0];
		vel_y=Vehicle_velocity[1];
		vel_z=Vehicle_velocity[2];
		rot_x=Vehicle_velocity[3];
		rot_y=Vehicle_velocity[4];
		rot_z=Vehicle_velocity[5];

 		float data_list[13]={0.0,0.0,0.0,0.0,0.0,0.0,1,vel_x,vel_y,vel_z,rot_x,rot_y,rot_z};
		odom = setOdometry_Information(odom,data_list);

		return odom;
	}

//Returns the joint state values based on the input values.
sensor_msgs::JointState setJointValues(float slew,float shoulder,float elbow,float jawrot,float jawopen)
	{
		sensor_msgs::JointState jointstatevalues;
		jointstatevalues.name.push_back(std::string("Slew"));
		jointstatevalues.position.push_back(slew);
		jointstatevalues.name.push_back(std::string("Shoulder"));
		jointstatevalues.position.push_back(shoulder);
		jointstatevalues.name.push_back(std::string("Elbow"));
		jointstatevalues.position.push_back(elbow);
		jointstatevalues.name.push_back(std::string("JawRotate"));
		jointstatevalues.position.push_back(jawrot);
		jointstatevalues.name.push_back(std::string("JawOpening"));
		jointstatevalues.position.push_back(jawopen);
		return jointstatevalues;
	}

