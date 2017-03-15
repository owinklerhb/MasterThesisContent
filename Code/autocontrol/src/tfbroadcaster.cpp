#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

int main(int argc, char** argv)
{	
	std::string veh_name;
	veh_name = argv[1];
	ros::init(argc, argv,"vehicle_tf_broadcaster");
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "girona500"));
	ros::spin();
	return 0;
};


