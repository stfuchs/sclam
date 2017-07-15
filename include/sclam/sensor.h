#ifndef SCLAM_SENSOR_H__
#define SCLAM_SENSOR_H__

#include <array>
#include <algorithm>
#include <Eigen/Geometry>

#include <sclam/pose.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>

class Sensor
{
public:
  typedef std::shared_ptr<Sensor> Ptr;
  typedef std::shared_ptr<const Sensor> ConstPtr;

  enum TypeId { TYPE_ID_SE3, TYPE_ID_POINT3, TYPE_ID_COUNT, TYPE_ID_UNDEF };
  static const std::array<const std::string, 2> type_names;

  static TypeId parseType(const std::string& str)
  {
    auto res = std::find(type_names.begin(), type_names.end(), str);
    if (res!=type_names.end()) { return static_cast<TypeId>(res - type_names.begin()); }
    return TYPE_ID_UNDEF;
  }

  struct SE3
  {
    typedef g2o::VertexSE3 vertex_type;
    typedef g2o::EdgeSE3 edge_type;
    typedef g2o::ParameterSE3Offset parameter_type;

    inline static Eigen::Isometry3d get_estimate(const Pose::ConstPtr& pose)
    {
      return pose->transform();
    }

    inline static const Eigen::Matrix6d& get_information(const Pose::ConstPtr& pose)
    {
      return pose->information();
    }
  };

  struct POINT3
  {
    typedef g2o::VertexPointXYZ vertex_type;
    typedef g2o::EdgeSE3PointXYZ edge_type;
    typedef g2o::ParameterSE3Offset parameter_type;

    inline static const Eigen::Vector3d& get_estimate(const Pose::ConstPtr& pose)
    {
      return pose->translation();
    }

    inline static Eigen::Matrix3d get_information(const Pose::ConstPtr& pose)
    {
      return pose->information().topLeftCorner<3,3>();
    }
  };

  Sensor(const std::string& frame_id, TypeId type, const Pose::ConstPtr& offset)
    : _id(-1)
    , _frame_id(frame_id)
    , _type(type)
    , _offset(*offset) {}

  ~Sensor() {}

  inline int id() const { return _id; }
  inline int id(int new_id) { _id = new_id; return _id; }
  inline const std::string& frameId() const { return _frame_id; }
  inline TypeId typeId() const { return _type; }
  inline const std::string& typeName() const { return Sensor::type_names[_type]; }
  inline Eigen::Isometry3d offset() const { return _offset.transform(); }

private:
  int _id;
  std::string _frame_id;
  TypeId _type;
  Pose _offset;
};

const std::array<const std::string,2> Sensor::type_names = {"SE3","POINT3"};

#endif
