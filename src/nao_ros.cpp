// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <raisim_ros/raisim_tools.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "raisim_ros/Queue.h"

using namespace std::chrono;
using namespace Eigen;
//Simulation Step, 0.01 is the hardware loop of the actual NAO
double dt = 0.01;
double freq = 1.0 / dt;
int micro_dt = dt * 1000000;
Queue<sensor_msgs::JointStateConstPtr> joint_data;

void commandJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  joint_data.push(msg);
  if (joint_data.size() > (int)freq / 10)
    joint_data.pop();
}

int main(int argc, char *argv[])
{
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey("/home/master/raisim_workspace/raisim_examples/rsc/activation.raisim");
#if WIN32
  timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif

  /// create raisim world
  raisim::World world;
  world.setTimeStep(dt);
  world.setERP(0, 0);
  //world.setGravity(Eigen::Vector3d(0,0,0));

  /// create objects
  auto ground = world.addGround();
  auto NAO = world.addArticulatedSystem("/home/master/raisim_workspace/raisim_examples/rsc/nao/nao.urdf");

  ///Remove ROOT + universe joints names from the joint name vector get only the actuated joints
  std::vector<std::string> jnames = NAO->getMovableJointNames();
  auto itr = std::find(jnames.begin(), jnames.end(), "ROOT");
  if (itr != jnames.end())
    jnames.erase(itr);

  auto itr_ = std::find(jnames.begin(), jnames.end(), "universe");
  if (itr_ != jnames.end())
    jnames.erase(itr_);

  cout << "Joint Names " << endl;
  for (int i = 0; i < jnames.size(); i++)
    cout << jnames[i] << endl;
  ;

  Eigen::VectorXd jointNominalConfig, jointNominalVelocity;

  ///Set Nominal Configuration
  jointNominalConfig.resize(NAO->getGeneralizedCoordinateDim());
  jointNominalVelocity.resize(NAO->getDOF());
  jointNominalConfig.setZero();
  jointNominalConfig << 0, 0, 0.32, 1, 0, 0, 0,
      0.0, 0.0,
      0.0, 0.0, -0.3976, 0.85, -0.4427, -0.009,
      0.0, 0.0, -0.3976, 0.85, -0.4427, -0.009,
      1.5, 0.15, 0, -0.0349066, -1.5, 0,
      1.5, -0.15, 0, 0.0349066, 1.5, 0;

  ///Set Joint PD Gains
  Eigen::VectorXd jointPgain(NAO->getDOF()), jointDgain(NAO->getDOF());
  jointPgain.setConstant(350);
  jointDgain.setConstant(5); //Gazebo D gain is 0.1 but makes NAO unstable in raisim

  ///Set the Initial Configuration in the world
  NAO->setGeneralizedCoordinate(jointNominalConfig);
  NAO->setGeneralizedVelocity(jointNominalVelocity);
  NAO->setGeneralizedForce(Eigen::VectorXd::Zero(NAO->getDOF()));
  NAO->setPdGains(jointPgain, jointDgain);
  NAO->setName("NAO");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(NAO);

  /// q, dq is the actual configuration from raisim
  Eigen::VectorXd q(NAO->getGeneralizedCoordinateDim()), dq(NAO->getDOF());

  /// Set Desired Frames for the WBC
  string lfoot_frame = "l_sole";
  string rfoot_frame = "r_sole";
  string base_frame = "base_link";
  string lhand_frame = "LHand";
  string rhand_frame = "RHand";
  string head_frame = "Head";

  Vector3d CoM_pos_ref, CoM_vel_ref, CoM_acc_ref, CoM_vel_ref_, lhand_pos_ref, rhand_pos_ref, lhand_linear_vel_ref, rhand_linear_vel_ref, head_pos_ref, head_linear_vel_ref,
      lf_pos_ref, rf_pos_ref, lf_linear_vel_ref, rf_linear_vel_ref, lf_angular_vel_ref, rf_angular_vel_ref, base_pos_ref, base_linear_vel_ref, base_angular_vel_ref,
      lhand_angular_vel_ref, rhand_angular_vel_ref, head_angular_vel_ref;
  Vector3d CoM_pos, CoM_vel, CoM_vel_, CoM_acc;
  Quaterniond lf_orientation_ref, rf_orientation_ref, base_orientation_ref, head_orientation_ref, lhand_orientation_ref, rhand_orientation_ref;
  VectorXd joint_states_ref;

  double mass = NAO->getTotalMass();
  // set PD TODO

  int init_idx = 0;
  bool LSS, RSS, DS;
  bool firstCoMVel = true;

  /// For Contact Detection, COP, GRF and GRT computation
  auto RfootIndex = NAO->getBodyIdx("r_ankle");
  auto LfootIndex = NAO->getBodyIdx("l_ankle");
  auto RfootFrameIndex = NAO->getFrameIdxByName("RLeg_effector_fixedjoint");
  auto LfootFrameIndex = NAO->getFrameIdxByName("LLeg_effector_fixedjoint");
  Eigen::Vector3d RLegGRF, RLegGRT, LLegGRF, LLegGRT, footForce, footTorque, LLeg_COP, RLeg_COP;
  raisim::Vec<3> footPosition;
  raisim::Mat<3, 3> footOrientation;
  Affine3d Tir, Til;
  Eigen::Vector3d RfootPosition;
  Eigen::Matrix3d RfootOrientation;
  Eigen::Vector3d LfootPosition;
  Eigen::Matrix3d LfootOrientation;
  Quaterniond qir, qil;
  Vector3d COPL, COPR, ZMP;

  double dist_t = 0;
  int idx = 0;
  int start_idx = 500;

  // INIT ROS PUBLISHER

  ros::init(argc, argv, "nao_ros_publisher");
  ros::NodeHandle n;

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/nao_raisim_ros/joint_states", 1000);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/nao_raisim_ros/odom", 1000);
  ros::Publisher wrench_pub_LLeg = n.advertise<geometry_msgs::WrenchStamped>("/nao_raisim_ros/LLeg/force_torque_states", 1000);
  ros::Publisher wrench_pub_RLeg = n.advertise<geometry_msgs::WrenchStamped>("/nao_raisim_ros/RLeg/force_torque_states", 1000);
  ros::Publisher gait_phase_pub = n.advertise<std_msgs::String>("/nao_raisim_ros/gait_phase", 1000);
  ros::Publisher com_pub = n.advertise<nav_msgs::Odometry>("/nao_raisim_ros/CoM", 1000);
  ros::Publisher odom_pub_LLeg = n.advertise<nav_msgs::Odometry>("/nao_raisim_ros/LLeg/odom", 1000);
  ros::Publisher odom_pub_RLeg = n.advertise<nav_msgs::Odometry>("/nao_raisim_ros/RLeg/odom", 1000);
  ros::Publisher zmp_pub = n.advertise<geometry_msgs::PointStamped>("/nao_raisim_ros/ZMP", 1000);
  ros::Subscriber command_js_sub = n.subscribe("/nao_raisim_ros/command_joint_states", 1000, commandJointStatesCallback);
  ros::Rate loop_rate(120);

  while (ros::ok())
  {

    auto start = high_resolution_clock::now();

    NAO->getState(q, dq);

    /// Get Leg Positions in raisim
    NAO->getFramePosition(RfootFrameIndex, footPosition);
    NAO->getFrameOrientation(RfootFrameIndex, footOrientation);
    RfootPosition = Eigen::Vector3d(footPosition[0], footPosition[1], footPosition[2]);
    RfootOrientation << footOrientation[0], footOrientation[1], footOrientation[2], footOrientation[3], footOrientation[4], footOrientation[5], footOrientation[6], footOrientation[7], footOrientation[8];
    //std::cout<<"Right Foot Pos"<< RfootPosition.transpose()<<std::endl;

    Tir.translation() = RfootPosition;
    Tir.linear() = RfootOrientation;
    qir = Quaterniond(RfootOrientation);

    NAO->getFramePosition(LfootFrameIndex, footPosition);
    NAO->getFrameOrientation(LfootFrameIndex, footOrientation);
    LfootPosition = Eigen::Vector3d(footPosition[0], footPosition[1], footPosition[2]);
    //std::cout<<"Left Foot Pos"<< LfootPosition.transpose()<<std::endl;

    LfootOrientation << footOrientation[0], footOrientation[1], footOrientation[2], footOrientation[3], footOrientation[4], footOrientation[5], footOrientation[6], footOrientation[7], footOrientation[8];
    Til.translation() = LfootPosition;
    Til.linear() = LfootOrientation;
    qil = Quaterniond(LfootOrientation);

    //std::cout<<"Left Foot Pos"<< LfootOrientation<<std::endl;

    RLegGRT.setZero();
    LLegGRT.setZero();
    RLegGRF.setZero();
    LLegGRF.setZero();
    RSS = false;
    LSS = false;
    DS = false;
    COPL.setZero();
    COPR.setZero();
    /// for all contacts on the robot, check ...
    for (auto &contact : NAO->getContacts())
    {
      if (contact.skip())
        continue; /// if the contact is internal, one contact point is set to 'skip'
      if (RfootIndex == contact.getlocalBodyIndex() && fabs(contact.getPosition().e()(2) - RfootPosition(2)) < 0.015)
      {
        footForce = contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
        RLegGRF += footForce;
        COPR += contact.getPosition().e() * footForce(2);
        footTorque = (contact.getPosition().e() - RfootPosition).cross(footForce);
        RLegGRT += footTorque;
        //cout<<"Right Contact Point "<<(contact.getPosition().e() ).transpose()<<" "<<contact.isObjectA()<<endl;
        if (contact.isObjectA())
          RSS = true;
      }
      if (LfootIndex == contact.getlocalBodyIndex() && fabs(contact.getPosition().e()(2) - LfootPosition(2)) < 0.015)
      {
        footForce = contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
        LLegGRF += footForce;
        //cout<<"Left Contact Point "<<(contact.getPosition().e()).transpose()<<" "<<contact.isObjectA()<<endl;
        COPL += contact.getPosition().e() * footForce(2);
        footTorque = (contact.getPosition().e() - LfootPosition).cross(footForce);
        LLegGRT += footTorque;
        if (contact.isObjectA())
          LSS = true;
      }
    }

    if (LSS && RSS)
      DS = true;

    if (LLegGRF(2) > 7)
    {
      LLeg_COP = Eigen::Vector3d(-LLegGRT(1) / LLegGRF(2), LLegGRT(0) / LLegGRF(2), 0) + LfootPosition;
      COPL = COPL / LLegGRF(2);
      COPL(2) = LfootPosition(2);
    }
    else
    {
      LLeg_COP.setZero();
      COPL.setZero();
    }
    if (RLegGRF(2) > 7)
    {
      RLeg_COP = Eigen::Vector3d(-RLegGRT(1) / RLegGRF(2), RLegGRT(0) / RLegGRF(2), 0) + RfootPosition;
      COPR = COPR / RLegGRF(2);
      COPR(2) = RfootPosition(2);
    }
    else
    {
      RLeg_COP.setZero();
      COPR.setZero();
    }

    // cout<<"COP L "<<COPL.transpose()<<" "<<LLeg_COP.transpose()<<endl;
    // cout<<"COP R "<<COPR.transpose()<<" "<<RLeg_COP.transpose()<<endl;
    raisim::Vec<3> linear_momentum = NAO->getLinearMomentum();
    raisim::Vec<3> center_of_mass = NAO->getCOM();

    ZMP = COPR + COPL;
    ZMP(2) /= 2.0;
    CoM_vel = Vector3d(linear_momentum(0), linear_momentum(1), linear_momentum(2)) / mass;
    CoM_pos = Vector3d(center_of_mass(0), center_of_mass(1), center_of_mass(2));

    cout << "Buffer Size " << joint_data.size() << endl;

    if (joint_data.size() > 0)
    {
      sensor_msgs::JointStateConstPtr msg = joint_data.pop();
      std::vector<double> pos_vector = msg->position;
      double *pos_array = pos_vector.data();
      jointNominalConfig = Eigen::Map<Eigen::Matrix<double, 33, 1>>(pos_array);

      std::vector<double> vel_vector = msg->velocity;
      double *vel_array = vel_vector.data();
      jointNominalVelocity = Eigen::Map<Eigen::Matrix<double, 32, 1>>(vel_array);
    }
    NAO->setPdTarget(jointNominalConfig, jointNominalVelocity);

    //NAO->setGeneralizedCoordinate(jointNominalConfig);
    //NAO->setGeneralizedVelocity(jointNominalVelocity);

    ///////////////////////////////// PUBLISHING ///////////////////

    // JOINT PUBLISHER
    sensor_msgs::JointState joint_msg;

    // from Eigen:vectorXd to pub msg
    std::vector<double> qfoo(q.data() + 7, q.data() + q.size()); // convert  e to std vector
    joint_msg.position.resize(qfoo.size());                      // TODO fix small size
    joint_msg.position = qfoo;

    std::vector<double> dqfoo(dq.data() + 6, dq.data() + dq.size()); // convert  e to std vector
    joint_msg.velocity.resize(dqfoo.size());                         // TODO fix small size
    joint_msg.velocity = dqfoo;

    joint_msg.name.resize(jnames.size());
    joint_msg.name = jnames;

    joint_msg.header.stamp = ros::Time::now();

    joint_pub.publish(joint_msg);

    // ODOM PUBLISHER

    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x = q(0);
    odom_msg.pose.pose.position.y = q(1);
    odom_msg.pose.pose.position.z = q(2);
    odom_msg.pose.pose.orientation.x = q(4);
    odom_msg.pose.pose.orientation.y = q(5);
    odom_msg.pose.pose.orientation.z = q(6);
    odom_msg.pose.pose.orientation.w = q(3);

    odom_msg.twist.twist.linear.x = dq(0);
    odom_msg.twist.twist.linear.y = dq(1);
    odom_msg.twist.twist.linear.z = dq(2);
    odom_msg.twist.twist.angular.x = dq(3);
    odom_msg.twist.twist.angular.y = dq(4);
    odom_msg.twist.twist.angular.z = dq(5);

    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "base_link";

    odom_msg.header.stamp = ros::Time::now();

    odom_pub.publish(odom_msg);

    //RLeg Odom Publisher
    nav_msgs::Odometry rodom_msg;

    rodom_msg.pose.pose.position.x = RfootPosition(0);
    rodom_msg.pose.pose.position.y = RfootPosition(1);
    rodom_msg.pose.pose.position.z = RfootPosition(2);
    rodom_msg.pose.pose.orientation.x = qir.x();
    rodom_msg.pose.pose.orientation.y = qir.y();
    rodom_msg.pose.pose.orientation.z = qir.z();
    rodom_msg.pose.pose.orientation.w = qir.w();

    rodom_msg.header.frame_id = "world";
    rodom_msg.child_frame_id = "r_sole";

    rodom_msg.header.stamp = ros::Time::now();

    odom_pub_RLeg.publish(rodom_msg);

    //LLeg Odom Publisher
    nav_msgs::Odometry lodom_msg;

    lodom_msg.pose.pose.position.x = LfootPosition(0);
    lodom_msg.pose.pose.position.y = LfootPosition(1);
    lodom_msg.pose.pose.position.z = LfootPosition(2);
    lodom_msg.pose.pose.orientation.x = qil.x();
    lodom_msg.pose.pose.orientation.y = qil.y();
    lodom_msg.pose.pose.orientation.z = qil.z();
    lodom_msg.pose.pose.orientation.w = qil.w();

    lodom_msg.header.frame_id = "world";
    lodom_msg.child_frame_id = "l_sole";

    lodom_msg.header.stamp = ros::Time::now();

    odom_pub_LLeg.publish(lodom_msg);

    // FORCE TORQUE PUBLISHER

    geometry_msgs::WrenchStamped wrench_msg;

    wrench_msg.wrench.force.x = LLegGRF(0);
    wrench_msg.wrench.force.y = LLegGRF(1);
    wrench_msg.wrench.force.z = LLegGRF(2);

    wrench_msg.wrench.torque.x = LLegGRT(0);
    wrench_msg.wrench.torque.y = LLegGRT(1);
    wrench_msg.wrench.torque.z = LLegGRT(2);

    wrench_msg.header.frame_id = "world";

    wrench_msg.header.stamp = ros::Time::now();
    wrench_pub_LLeg.publish(wrench_msg);

    wrench_msg.wrench.force.x = RLegGRF(0);
    wrench_msg.wrench.force.y = RLegGRF(1);
    wrench_msg.wrench.force.z = RLegGRF(2);

    wrench_msg.wrench.torque.x = RLegGRT(0);
    wrench_msg.wrench.torque.y = RLegGRT(1);
    wrench_msg.wrench.torque.z = RLegGRT(2);

    wrench_msg.header.frame_id = "world";

    wrench_msg.header.stamp = ros::Time::now();
    wrench_pub_RLeg.publish(wrench_msg);

    // GAIT PHASE PUBLISHER

    std_msgs::String gait_msg;

    if (DS)
      gait_msg.data = "double_support";
    else if (LSS)
      gait_msg.data = "left_support";
    else if (RSS)
      gait_msg.data = "right_support";
    else
      gait_msg.data = "flight";

    gait_phase_pub.publish(gait_msg);

    // CENTER OF MASS PUBLISHER

    nav_msgs::Odometry com_msg;
    com_msg.pose.pose.position.x = CoM_pos(0);
    com_msg.pose.pose.position.y = CoM_pos(1);
    com_msg.pose.pose.position.z = CoM_pos(2);

    com_msg.twist.twist.linear.x = CoM_vel(0);
    com_msg.twist.twist.linear.y = CoM_vel(1);
    com_msg.twist.twist.linear.z = CoM_vel(2);

    com_msg.header.frame_id = "world";
    com_msg.child_frame_id = "com";

    com_msg.header.stamp = ros::Time::now();

    com_pub.publish(com_msg);

    // ZERO MOMENT POINT PUBLISHER
    geometry_msgs::PointStamped zmp_msg;

    zmp_msg.point.x = ZMP(0);
    zmp_msg.point.y = ZMP(1);
    zmp_msg.point.z = ZMP(2);

    zmp_msg.header.frame_id = "world";

    zmp_msg.header.stamp = ros::Time::now();

    zmp_pub.publish(zmp_msg);

    ////////////////////////////////// INTERGRATE ///////////////////////

    server.integrateWorldThreadSafe();

    // auto stop = high_resolution_clock::now();
    // auto duration = duration_cast<microseconds>(stop - start);
    // int loop_duration = micro_dt - duration.count();
    //
    // if(loop_duration>0)
    //   std::this_thread::sleep_for(std::chrono::microseconds(loop_duration));

    ros::spinOnce();

    loop_rate.sleep();
  }

  server.killServer();
}
