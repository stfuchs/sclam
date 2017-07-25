#ifndef SCLAM_VISUALIZATION_H__
#define SCLAM_VISUALIZATION_H__

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sclam/pose.h>

template<typename VertexT>
visualization_msgs::Marker toMarker(const VertexT* v, const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = v->id();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05; // arrow length
  marker.scale.y = 0.005; // arrow width
  marker.scale.z = 0.005; // arrow height
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  if (dynamic_cast<const g2o::VertexSE3*>(v))
  {
    toROS(dynamic_cast<const g2o::VertexSE3*>(v)->estimate(), marker.pose);
  }
  else if (dynamic_cast<const g2o::VertexPointXYZ*>(v))
  {
    toROS(dynamic_cast<const g2o::VertexPointXYZ*>(v)->estimate(), marker.pose.position);
    marker.color.g = 1.0;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to determine vertex type");
  }
  return marker;
}



#endif
