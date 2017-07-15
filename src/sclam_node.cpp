#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Trigger.h>

#include <sclam/Observation.h>
#include <sclam/sclam.h>

class SclamNode
{
public:
  SclamNode()
    : sclam_()
    , active_(false)
    , nh_()
    , nh_priv_("~")
  {}

  ~SclamNode() {}

  bool configure()
  {
    ROS_INFO("%s::configure() requested.", nh_priv_.getNamespace().c_str());
    sclam_.reset(new Sclam());

    nh_priv_.param<std::string>(
      "filename_graph_before", filename_before_, "/tmp/before.g2o");
    nh_priv_.param<std::string>(
      "filename_graph_after", filename_after_, "/tmp/after.g2o");
    nh_priv_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
    nh_priv_.param<std::map<std::string,std::string> >("sensors", sensors_);

    tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(10)));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

    ros::Rate(0.5).sleep(); // make the buffer fill up

    std::vector<Sensor::Ptr> sensors;

    for (auto it=sensors_.begin(); it!=sensors_.end(); ++it)
    {
      Sensor::TypeId tid = Sensor::parseType(it->second);
      if (tid != Sensor::TYPE_ID_UNDEF)
      {
        Pose::Ptr pose;
        if (_tfLookup(it->first, pose))
        {
          sensors.push_back(Sensor::Ptr(new Sensor(it->first, tid, pose)));
        }
      }
      else
      {
        ROS_ERROR_STREAM("Invalid sensor_type: " << it->second);
      }
    }

    sclam_->init(sensors);

    sub_obs_ = nh_.subscribe("observation", 10, &SclamNode::_cbSub, this);
    srv_optimize_ = nh_priv_.advertiseService("optimize", &SclamNode::_cbSrv, this);

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
    active_ = false;
    sub_obs_.shutdown();
    tf_listener_.reset();
    tf_buffer_.reset();
    sclam_.reset();
    sensors_.clear();
    return true;
  }

private:
  void _cbSub(const sclam::Observation& msg)
  {
    if (!active_) return;

    sclam_->update(msg.robot,msg.measurement,msg.header.frame_id,msg.landmark_id);
  }

  bool _cbSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    if (!active_)
    {
      res.success = false;
      return res.success;
    }
    sclam_->optimize(filename_before_, filename_after_);
    res.success = true;
    return res.success;
  }

  bool _tfLookup(const std::string& source_frame_id, Pose::Ptr& pose) const
  {
    geometry_msgs::Transform tf;
    try
    {
      tf = tf_buffer_->lookupTransform(
        base_frame_id_, // target frame
        source_frame_id, // source frame
        ros::Time(0)).transform;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    pose.reset(new Pose(tf));
    return true;
  }

  std::unique_ptr<Sclam> sclam_;

  bool active_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string filename_before_;
  std::string filename_after_;
  std::string base_frame_id_;
  std::map<std::string,std::string> sensors_;

  ros::Subscriber sub_obs_;
  ros::ServiceServer srv_optimize_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sclam_node");
  SclamNode node;
  node.configure();
  node.activate();
  ros::spin();
};
