#ifndef SCLAM_SCLAM_H__
#define SCLAM_SCLAM_H__

#include <unordered_map>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sclam/pose.h>
#include <sclam/sensor.h>

class Sclam
{
public:
  Sclam()
    : _last_id(-1)
    , _last_robot(new Pose)
    , _curr_robot(new Pose)
  {}

  ~Sclam()
  {
    // freeing the graph memory
    _optimizer.clear();

    // destroy all the singletons
    // Factory::destroy();
    // OptimizationAlgorithmFactory::destroy();
    // HyperGraphActionLibrary::destroy();
  }

  void init(const std::vector<Sensor::Ptr>& sensors)
  {
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SclamBlockSolver;
    typedef g2o::LinearSolverCSparse<SclamBlockSolver::PoseMatrixType> SclamLinearSolver;

    // allocate optimizer (optimizer takes over ownership of resources)
    SclamLinearSolver* linear_solver = new SclamLinearSolver();
    linear_solver->setBlockOrdering(false);
    SclamBlockSolver* block_solver = new SclamBlockSolver(linear_solver);
    g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(block_solver);
    _optimizer.setAlgorithm(solver);

    for (auto it=sensors.begin(); it!=sensors.end(); ++it)
    {
      _sensors[(*it)->frameId()] = *it;
      g2o::ParameterSE3Offset* sensor_offset = new g2o::ParameterSE3Offset;
      sensor_offset->setOffset((*it)->offset());
      sensor_offset->setId((*it)->id(++_last_id));
      _optimizer.addParameter(sensor_offset);
    }

    g2o::VertexSE3* robot_node = new g2o::VertexSE3;
    robot_node->setId(_last_robot->id(++_last_id));
    robot_node->setEstimate(_last_robot->transform());
    robot_node->setFixed(true);
    _optimizer.addVertex(robot_node);
  }

  void update(const geometry_msgs::PoseWithCovariance& robot_msg,
              const geometry_msgs::PoseWithCovariance& landm_msg,
              const std::string& frame_id,
              int landm_id)
  {
    Sensor::Ptr sensor = _sensors[frame_id];
    if (!sensor)
    {
      ROS_ERROR_STREAM_THROTTLE(
        1.0, "Requested update for unknown sensor: " << frame_id);
      return;
    }

    // odom
    _curr_robot.reset(new Pose(robot_msg));
    g2o::VertexSE3* robot_node = new g2o::VertexSE3;
    robot_node->setId(_curr_robot->id(++_last_id));
    robot_node->setEstimate(_curr_robot->transform());
    _optimizer.addVertex(robot_node);

    g2o::EdgeSE3* odometry = new g2o::EdgeSE3;
    odometry->vertices()[0] = _optimizer.vertex(_last_robot->id());
    odometry->vertices()[1] = _optimizer.vertex(_curr_robot->id());
    odometry->setMeasurement(_last_robot->transform().inverse() * _curr_robot->transform());
    odometry->setInformation(_curr_robot->information());
    _optimizer.addEdge(odometry);

    // observation
    Pose::Ptr curr_obs(new Pose(landm_msg));
    switch( sensor->typeId() )
    {
    case Sensor::TYPE_ID_SE3:
      _add<Sensor::SE3>(sensor, _landmarks[landm_id], _curr_robot, curr_obs);
      break;
    case Sensor::TYPE_ID_POINT3:
      _add<Sensor::POINT3>(sensor, _landmarks[landm_id], _curr_robot, curr_obs);
      break;
    default:
      // this should not happen
      ROS_ERROR_STREAM("Unsupported sensor type: " << sensor->typeName());
      return;
    }

    std::swap(_last_robot, _curr_robot);
    return;
  }

  void optimize(const std::string& filename_before,
                const std::string& filename_after)
  {
    ROS_INFO_STREAM("Optimizing. Saving initial state to " << filename_before);
    _optimizer.save(filename_before.c_str());
    _optimizer.setVerbose(true);
    _optimizer.initializeOptimization();
    _optimizer.optimize(10);

    ROS_INFO_STREAM("Done. Saving result state to " << filename_after);
    _optimizer.save(filename_after.c_str());
  }

private:
  template<typename SensorT>
  void _add(const Sensor::ConstPtr& sensor,
            const Pose::ConstPtr& robot,
            const Pose::ConstPtr& measurement,
            Pose::Ptr& landmark)
  {
    if (!landmark)
    {
      landmark.reset(new Pose(robot->transform() * measurement->transform()));
      typename SensorT::vertex_type* landmark_node = new typename SensorT::vertex_type;
      landmark_node->setId(landmark->id(++_last_id));
      landmark_node->setEstimate(SensorT::get_estimate(landmark));
      _optimizer.addVertex(landmark_node);
    }

    typename SensorT::edge_type* observation = new typename SensorT::edge_type;
    observation->vertices()[0] = _optimizer.vertex(robot->id());
    observation->vertices()[1] = _optimizer.vertex(landmark->id());
    observation->setMeasurement(SensorT::get_estimate(measurement));
    observation->setInformation(SensorT::get_information(measurement));
    observation->setParameterId(0, sensor->id());
    _optimizer.addEdge(observation);
  }

private:
  int _last_id;
  g2o::SparseOptimizer _optimizer;
  Pose::Ptr _last_robot;
  Pose::Ptr _curr_robot;
  std::unordered_map<int, Pose::Ptr > _landmarks;
  std::unordered_map<std::string, Sensor::Ptr > _sensors;
};

#endif
