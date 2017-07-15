#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <sclam/Observation.h>

class AlvarMarkerObserverNode
{
  typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry,
    ar_track_alvar_msgs::AlvarMarkers > SyncPolicy;

public:
  AlvarMarkerObserverNode()
    : active_(false)
    , nh_()
    , nh_priv_("~")
  {}

  ~AlvarMarkerObserverNode()
  {}

  bool configure()
  {
    ROS_INFO("%s::configure() requested.", nh_priv_.getNamespace().c_str());

    pub_ = nh_.advertise<sclam::Observation>("observation", 1);
    sub_odom_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odom", 1));
    sub_markers_.reset(
      new message_filters::Subscriber<ar_track_alvar_msgs::AlvarMarkers>(nh_, "alvar_markers", 1));
    sync_.reset(
      new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100), *sub_odom_, *sub_markers_));
    sync_->registerCallback(std::bind(&AlvarMarkerObserverNode::_cb, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));
    return true;
  }

  bool activate()
  {
    ROS_INFO("%s::activate() requested.", nh_priv_.getNamespace().c_str());
    active_ = true;
    return active_;
  }

  bool deactivate()
  {
    ROS_INFO("%s::deactivate() requested.", nh_priv_.getNamespace().c_str());
    active_ = false;
    return !active_;
  }

  bool cleanup()
  {
    ROS_INFO("%s::cleanup() requested.", nh_priv_.getNamespace().c_str());
    sync_.reset();
    sub_odom_.reset();
    sub_markers_.reset();
    return true;
  }

private:
  void _cb(const nav_msgs::OdometryConstPtr& odom,
           const ar_track_alvar_msgs::AlvarMarkersConstPtr& markers)
  {
    if (!active_) return;

    for (auto m_it=markers->markers.begin(); m_it!=markers->markers.end(); ++m_it)
    {
      sclam::Observation obs;
      obs.header = markers->header;
      obs.landmark_id = m_it->id;

      obs.measurement.pose = m_it->pose.pose;
      obs.measurement.covariance[ 0] = 0.01;
      obs.measurement.covariance[ 1] = 0;
      obs.measurement.covariance[ 2] = 0;
      obs.measurement.covariance[ 3] = 0;
      obs.measurement.covariance[ 4] = 0;
      obs.measurement.covariance[ 5] = 0;

      obs.measurement.covariance[ 6] = 0;
      obs.measurement.covariance[ 7] = 0.01;
      obs.measurement.covariance[ 8] = 0;
      obs.measurement.covariance[ 9] = 0;
      obs.measurement.covariance[10] = 0;
      obs.measurement.covariance[11] = 0;

      obs.measurement.covariance[12] = 0;
      obs.measurement.covariance[13] = 0;
      obs.measurement.covariance[14] = 0.01;
      obs.measurement.covariance[15] = 0;
      obs.measurement.covariance[16] = 0;
      obs.measurement.covariance[17] = 0;

      obs.measurement.covariance[18] = 0;
      obs.measurement.covariance[19] = 0;
      obs.measurement.covariance[20] = 0;
      obs.measurement.covariance[21] = 1;
      obs.measurement.covariance[22] = 0;
      obs.measurement.covariance[23] = 0;

      obs.measurement.covariance[24] = 0;
      obs.measurement.covariance[25] = 0;
      obs.measurement.covariance[26] = 0;
      obs.measurement.covariance[27] = 0;
      obs.measurement.covariance[28] = 1;
      obs.measurement.covariance[29] = 0;

      obs.measurement.covariance[30] = 0;
      obs.measurement.covariance[31] = 0;
      obs.measurement.covariance[32] = 0;
      obs.measurement.covariance[33] = 0;
      obs.measurement.covariance[34] = 0;
      obs.measurement.covariance[35] = 1;

      obs.robot = odom->pose;

      pub_.publish(obs);
    }
  }

  bool active_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry> > sub_odom_;
  std::unique_ptr<message_filters::Subscriber<ar_track_alvar_msgs::AlvarMarkers> > sub_markers_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  ros::Publisher pub_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "alvar_marker_observer_node");
  AlvarMarkerObserverNode node;
  node.configure();
  node.activate();
  ros::spin();
};
