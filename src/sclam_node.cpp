#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

class RobotNode
{
public:
  RobotNode()
    : _id(-1), _q(0,0,0,1), _t(0,0,0) {}

  RobotNode(double x, double y, double z, double qx, double qy, double qz, double qw)
    : _id(-1), _q(qx,qy,qz,qw), _t(0,0,0) {}

  RobotNode(const geometry_msgs::Pose& pose)
    : _id(-1)
    , _q(pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z,
         pose.orientation.w)
    , _t(pose.position.x,
         pose.position.y,
         pose.position.z) {}

  ~RobotNode() {}

  int id() const { return _id; }

  const Eigen::Vector3d& translation() const { return _t; }
  const Eigen::Quaterniond& rotation() const { return _q; }

  Eigen::Vector3d operator * (const Eigen::Vector3d& v) const
  {
    return _q*v + _t;
  }

  g2o::VertexSE3* createVertex(int id)
  {
    _id = id;
    g2o::Isometry3D est;
    est = _q.toRotationMatrix();
    est.translation() = _t;

    g2o::VertexSE3* robot = new g2o::VertexSE3;
    robot->setId(_id);
    robot->setEstimate(est);
    return robot;
  }

private:
  int _id;
  Eigen::Quaterniond _q;
  Eigen::Vector3d _t;
};

class LandmarkNode
{
public:
  LandmarkNode()
    : _id(-1), _t(0,0,0) {}
  LandmarkNode(const geometry_msgs::Point& obs, const RobotNode& robot)
    : _id(-1), _t(robot * Eigen::Vector3d(obs.x, obs.y, obs.z)) {}

  ~LandmarkNode() {}

  int id() const { return _id; }
  const Eigen::Vector3d& translation() const { return _t; }

  g2o::VertexPointXYZ* createVertex(int id)
  {
    _id = id;
    g2o::VertexPointXYZ* landmark = new g2o::VertexPointXYZ;
    landmark->setId(_id);
    landmark->setEstimate(_t);
    return landmark;
  }

private:
  int _id;
  Eigen::Vector3d _t;
};

class Sclam
{
public:

  Sclam()
    : _last_id(-1)
    , _last_pose(new RobotNode)
    , _curr_pose(new RobotNode)
    , _landmarks(16) {}

  ~Sclam() {}

  void init()
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
    _optimizer.addVertex(_last_pose->createVertex(++_last_id));
  }

  void update_pose(const geometry_msgs::Pose& pose)
  {
    _curr_pose.reset(new RobotNode(pose));
  }

  void add_observation(const geometry_msgs::Point& pos, int id)
  {
    if (int(_landmarks.size()) <= id)
    {
      _landmarks.resize(id+1);
    }
    if (!_landmarks[id])
    {
      _landmarks[id].reset(new LandmarkNode(pos, *_curr_pose));
      _optimizer.addVertex(_landmarks[id]->createVertex(++_last_id));
    }

    _optimizer.addVertex(_curr_pose->createVertex(++_last_id));
    _last_pose = _curr_pose;

    g2o::EdgeSE3PointXYZ* observation = new g2o::EdgeSE3PointXYZ;
    observation->vertices()[0] = _optimizer.vertex(_last_pose->id());
    observation->vertices()[1] = _optimizer.vertex(_landmarks[id]->id());
    //observation->setMeasurement();
    //observation->setInformation();
    _optimizer.addEdge(observation);

    // odom
    return;
  }

private:
  int _last_id;
  g2o::SparseOptimizer _optimizer;
  std::shared_ptr<RobotNode> _last_pose;
  std::shared_ptr<RobotNode> _curr_pose;
  std::vector<std::shared_ptr<LandmarkNode> > _landmarks;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sclam_node");
  ros::NodeHandle nh;

  Sclam node;
  node.init();

  ros::spin();
};
