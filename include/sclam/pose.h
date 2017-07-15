#ifndef SCLAM_POSE_H__
#define SCLAM_POSE_H__

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Transform.h>

namespace Eigen
{
  typedef Matrix<double, 6, 6> Matrix6d;
}

class Pose
{
private:
  int _id;
  Eigen::Quaterniond _q;
  Eigen::Vector3d _t;
  Eigen::Matrix6d _sigma;

public:
  typedef std::shared_ptr<Pose> Ptr;
  typedef std::shared_ptr<const Pose> ConstPtr;

  Pose()
    : _id(-1)
    , _q(0,0,0,1)
    , _t(0,0,0)
    , _sigma(Eigen::Matrix6d::Identity())
  {}

  Pose(const geometry_msgs::PoseWithCovariance& pose)
    : _id(-1)
    , _q(pose.pose.orientation.x,
         pose.pose.orientation.y,
         pose.pose.orientation.z,
         pose.pose.orientation.w)
    , _t(pose.pose.position.x,
         pose.pose.position.y,
         pose.pose.position.z)
    , _sigma(Eigen::Map<const Eigen::Matrix6d>(&pose.covariance[0]).inverse())
  {}

  Pose(const geometry_msgs::Transform& tf)
    : _id(-1)
    , _q(tf.rotation.x,
         tf.rotation.y,
         tf.rotation.z,
         tf.rotation.w)
    , _t(tf.translation.x,
         tf.translation.y,
         tf.translation.z)
    , _sigma(Eigen::Matrix6d::Identity())
  {}

  Pose(const Eigen::Isometry3d& transform)
    : _id(-1)
    , _q(transform.rotation())
    , _t(transform.translation())
    , _sigma(Eigen::Matrix6d::Identity())
  {}

  ~Pose() {}

  // Return position in world
  inline const Eigen::Vector3d& translation() const { return _t; }

  // Return orienation in world
  inline const Eigen::Quaterniond& rotation() const { return _q; }

  // Return transform from observation to world
  inline Eigen::Isometry3d transform() const { return Eigen::Translation3d(_t) * _q; }

  // Return inverse covariance
  inline const Eigen::Matrix6d& information() const { return _sigma; }

  inline int id() const { return _id; }
  inline int id(int new_id) { _id = new_id; return _id; }
};


#endif
