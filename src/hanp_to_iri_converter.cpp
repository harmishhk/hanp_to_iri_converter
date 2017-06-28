// defining constants
#define NODE_NAME "hanp_to_iri_converter"
#define HUMANS_SUB_TOPIC "in_humans"
#define HUMANS_PUB_TOPIC "out_humans"
#define DEFAULT_SEGMENT hanp_msgs::TrackedSegmentType::TORSO

#include <hanp_msgs/TrackedHumans.h>
#include <hanp_msgs/TrackedSegmentType.h>
#include <iri_perception_msgs/detectionArray.h>
#include <ros/ros.h>
#include <signal.h>

class HANPToIRIConverter {
public:
  void initialize();

private:
  // ros subscribers and publishers
  ros::Subscriber humans_sub_;
  ros::Publisher humans_pub_;

  int default_segment_;

  void humansCB(const hanp_msgs::TrackedHumans &humans);
};

void HANPToIRIConverter::initialize() {
  // get private node handle
  ros::NodeHandle pnh("~/");

  // get parameters
  pnh.param("default_segment", default_segment_, (int)(DEFAULT_SEGMENT));

  // initialize subscribers and publishers
  humans_sub_ =
      pnh.subscribe(HUMANS_SUB_TOPIC, 1, &HANPToIRIConverter::humansCB, this);
  humans_pub_ =
      pnh.advertise<iri_perception_msgs::detectionArray>(HUMANS_PUB_TOPIC, 1);
}

void HANPToIRIConverter::humansCB(const hanp_msgs::TrackedHumans &humans) {
  ROS_INFO_ONCE_NAMED(NODE_NAME, "%s: got humans", NODE_NAME);

  iri_perception_msgs::detectionArray iri_humans;
  iri_humans.header.frame_id = "/map";
  iri_humans.header.stamp = ros::Time::now();

  for (auto &human : humans.humans) {
    for (auto &segment : human.segments) {
      if (segment.type == default_segment_) {
        iri_perception_msgs::detection iri_human;
        iri_human.id = human.track_id;
        iri_human.position = segment.pose.pose.position;
        iri_human.covariances = segment.pose.covariance;
        iri_human.velocity.x = segment.twist.twist.linear.x;
        iri_human.velocity.y = segment.twist.twist.linear.y;
        iri_human.velocity.z = segment.twist.twist.linear.z;
        iri_humans.detection.push_back(iri_human);
      }
    }
  }

  humans_pub_.publish(iri_humans);
}

// handler for something to do before killing the node
void sigintHandler(int sig) {
  ROS_DEBUG_NAMED(NODE_NAME, "%s: node will now shutdown", NODE_NAME);

  // the default sigint handler, it calls shutdown() on node
  ros::shutdown();
}

// the main method starts a ros-node and initializes the HANPToIRIConverter class
int main(int argc, char **argv) {
  // starting the hanp_head_behavior node
  ros::init(argc, argv, NODE_NAME);
  ROS_DEBUG_NAMED(NODE_NAME, "started %s node", NODE_NAME);

  // initializing HANPToIRIConverter class
  HANPToIRIConverter hANPToIRIConverter;
  hANPToIRIConverter.initialize();

  // look for sigint and start spinning the node
  signal(SIGINT, sigintHandler);
  ros::spin();

  return 0;
}
