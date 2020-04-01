#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

struct location {
	double x;
	double y;
	double theta;
};

location PICKUP = {2, -2, -1};
location DROPOFF = {-4, 1, 1};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "virtual_object";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (marker_pub.getNumSubscribers() < 1)
  {
	if (!ros::ok())
	{
	  return 0;
	}
	ROS_WARN_ONCE("Please create a subscriber to the marker");
	sleep(1);
  }

  while (ros::ok())
  {
    // Publish the marker
    ROS_INFO("Place virtual object in pickup site");
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = PICKUP.x;
    marker.pose.position.y = PICKUP.y;
    marker_pub.publish(marker);

    sleep(5);

    ROS_INFO("Remove virtual object from pickup site");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    sleep(5);

    ROS_INFO("Place virtual object in dropoff site");
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = DROPOFF.x;
    marker.pose.position.y = DROPOFF.y;
	marker_pub.publish(marker);

	sleep(5);

	ROS_INFO("Remove virtual object from dropoff site");
	marker.action = visualization_msgs::Marker::DELETE;
	marker_pub.publish(marker);

	sleep(5);
  }

  return 0;
}
