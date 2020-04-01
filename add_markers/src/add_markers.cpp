#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

struct location {
	double x;
	double y;
};

const location PICKUP = {2, -2};
const location DROPOFF = {-4, 1};

const double DISTANCE_THRESHOLD = 0.01;
const int WAIT_TIME = 5;

class VirtualObjectManager
{
public:
  VirtualObjectManager()
  {
    //marker publisher topic
	  marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	  while (marker_pub_.getNumSubscribers() < 1)
		{
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}

    //subscribe to odometry topic
    odom_sub_ = n_.subscribe("/odom", 1000, &VirtualObjectManager::callback, this);

    //Create Marker
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    virtual_object_.header.frame_id = "/map";
    virtual_object_.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    virtual_object_.ns = "virtual_object";
    virtual_object_.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    virtual_object_.type = visualization_msgs::Marker::SPHERE;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    virtual_object_.pose.position.z = 0;
    virtual_object_.pose.orientation.x = 0.0;
    virtual_object_.pose.orientation.y = 0.0;
    virtual_object_.pose.orientation.z = 0.0;
    virtual_object_.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    virtual_object_.scale.x = 0.3;
    virtual_object_.scale.y = 0.3;
    virtual_object_.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    virtual_object_.color.r = 0.0f;
    virtual_object_.color.g = 0.0f;
    virtual_object_.color.b = 1.0f;
    virtual_object_.color.a = 1.0;

    virtual_object_.lifetime = ros::Duration();

    // Put marker in pickup location
    put_virtual_object(PICKUP.x, PICKUP.y);
    state_ = atPickup;

  }

  void put_virtual_object(double x, double y)
  {
		ROS_INFO("Place virtual object in ([%f], [%f])", x, y);
		virtual_object_.action = visualization_msgs::Marker::ADD;
		virtual_object_.pose.position.x = x;
		virtual_object_.pose.position.y = y;
		marker_pub_.publish(virtual_object_);
  }

  void rm_virtual_object()
  {
  	ROS_INFO("Remove virtual object");
		virtual_object_.action = visualization_msgs::Marker::DELETE;
		marker_pub_.publish(virtual_object_);
  }

  void callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
  	location robot = {msg->pose.pose.position.x, msg->pose.pose.position.y};
  	switch (state_) {
  		case atPickup:
  		{
  			// Calculate distance to pickup site
  			double dist = sqrt(pow(robot.x - PICKUP.x, 2) + pow(robot.y - PICKUP.y, 2));
  			if (dist < DISTANCE_THRESHOLD) {
  				rm_virtual_object();
  				sleep(WAIT_TIME);
  				state_ = pickedUp;
  			}
  			break;
  		}
  		case pickedUp:
  		{
  			// Calculate distance to dropoff site
				double dist = sqrt(pow(robot.x - DROPOFF.x, 2) + pow(robot.y - DROPOFF.y, 2));
				if (dist < DISTANCE_THRESHOLD) {
					put_virtual_object(DROPOFF.x, DROPOFF.y);
					state_ = atDropoff;
				}
  			break;
  		}
  		case atDropoff:
  			// goal achieved
  			break;
  	}
  }

private:
  ros::NodeHandle n_;
  ros::Publisher marker_pub_;
  ros::Subscriber odom_sub_;
  visualization_msgs::Marker virtual_object_;
  enum State {atPickup, pickedUp, atDropoff} state_;  // virtual object state

};//End of class VirtualObjectManager


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "add_markers");

  //Create an object of class SubscribeAndPublish that will take care of everything
  VirtualObjectManager vo_manager;

  ros::spin();

  return 0;
}
