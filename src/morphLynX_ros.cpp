#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
#include <raisim_ros/raisim_tools.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "raisim_ros/Queue.h"


using namespace Eigen;
//Simulation Step, 0.01 is the hardware loop of the actual morphLynX
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


char fileExists(const char * filename)
{
    FILE *fp = fopen(filename,"r");
    if( fp )
        {
            /* exists */ 
            fclose(fp);
            return 1;
        }
    return 0;
}


int main(int argc, char *argv[])
{
  //Workaround because RAISIM doesn't like relative paths..
  //Our run_it.sh will tell us our path so we can convert the relative paths to absolute 
  const char * path=0;
  for (int i=0; i<argc; i++)
     {
       if (strcmp(argv[i],"--from")==0)
       {
          path = argv[i+1];
          ROS_INFO("Received ROS path to convert relative paths to absolute.."); 
          fprintf(stderr,"Bin path = %s\n",path); 
       }
     }
   //---------------------------------------------------------------------------------


  ros::init(argc, argv, "morphLynX_raisim_ros_publisher");
  ros::NodeHandle n;
  ros::NodeHandle n_p("~");

  string modelname, activation_key;
  bool enable_gravity, animation_mode;
  double jointPgain_, jointDgain_;

  n_p.param<std::string>("modelname",modelname,"../rsc/morphLynX/morphLynX.urdf");
  n_p.param<std::string>("activation_key",activation_key,"../rsc/activation.raisim");
  n_p.param<bool>("enable_gravity",enable_gravity,true);
  n_p.param<bool>("animation_mode",animation_mode,false);

  //If we received a path lets do the conversion..
  if (path!=0)
    {
     char fullPath[2048];
     snprintf(fullPath,2048,"%s/%s",path,modelname.c_str());
     string fullPathModelName(fullPath);
     ROS_INFO("Converted model path to .."); 
     fprintf(stderr,"Absolute path = %s\n",fullPath); 
     
     if (!fileExists(fullPath))
       {
         ROS_INFO("The absolute path we generated does not correspond to an existing file, we will not use it .."); 
       } else 
       {
         modelname = fullPathModelName;
       } 
    } 
  //----------------------------------------------

  n_p.param<double>("jointPgain", jointPgain_, 350);
  n_p.param<double>("jointDgain", jointDgain_, 5);

  ROS_INFO("Using RAISIM activation key.."); 
  raisim::World::setActivationKey(activation_key);
  ROS_INFO("Creating RAISIM world.."); 
  ///create raisim world
  raisim::World world;

  ROS_INFO("Populating world.."); 
  world.setTimeStep(dt);
  world.setERP(0, 0);

  if (!enable_gravity)
    world.setGravity(Eigen::Vector3d(0, 0, 0));
   


  ///create objects
  auto ground = world.addGround();
  auto morphLynX = world.addArticulatedSystem(modelname);

  ROS_INFO("Parsing joint names.."); 
  ///Remove ROOT + universe joints names from the joint name vector get only the actuated joints
  std::vector<std::string> jnames = morphLynX->getMovableJointNames();
  auto itr = std::find(jnames.begin(), jnames.end(), "ROOT");
  if (itr != jnames.end())
    jnames.erase(itr);

  auto itr_ = std::find(jnames.begin(), jnames.end(), "universe");
  if (itr_ != jnames.end())
    jnames.erase(itr_);

  cout << "Joint Names " << endl;
  for (int i = 0; i < jnames.size(); i++)
    cout << jnames[i] << endl;
  
  
  cout<<"Body Names"<<endl;
  morphLynX->printOutBodyNamesInOrder();
  cout<<"Frame Names"<<endl;
  morphLynX->printOutFrameNamesInOrder();

  Eigen::VectorXd jointNominalConfig(morphLynX->getGeneralizedCoordinateDim()), jointNominalVelocity(morphLynX->getDOF());

  ///Set Nominal Configuration
  jointNominalConfig.setZero();
  jointNominalVelocity.setZero();
  jointNominalConfig[2] = 0.166;
  jointNominalConfig[3] = 1;

  ///Set Joint PD Gains
  Eigen::VectorXd jointPgain(morphLynX->getDOF()), jointDgain(morphLynX->getDOF());
  jointPgain.setConstant(jointPgain_);
  jointDgain.setConstant(jointDgain_); //Gazebo D gain is 0.1 but makes morphLynX unstable in raisim

  ///Set the Initial Configuration in the world
  morphLynX->setGeneralizedCoordinate(jointNominalConfig);
  morphLynX->setGeneralizedVelocity(jointNominalVelocity);
  morphLynX->setPdGains(jointPgain, jointDgain);
  morphLynX->setName("morphLynX");

  ROS_INFO("Launching RAISIM server.."); 
  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(morphLynX);

  /// q, dq is the actual configuration from raisim
  Eigen::VectorXd q(morphLynX->getGeneralizedCoordinateDim()), dq(morphLynX->getDOF());

  /// Set Desired Frames for the WBC
  string lfoot_frame = "L-SUCTIONCUP";
  string rfoot_frame = "R-SUCTIONCUP";
  string base_frame = "base_link";


  double mass = morphLynX->getTotalMass();
  bool LSS, RSS, DS;
  bool firstCoMVel = true;

  /// For Contact Detection, COP, GRF and GRT computation
  auto RfootIndex = morphLynX->getBodyIdx("R-SERVO-FOOT-ROLL");
  auto LfootIndex = morphLynX->getBodyIdx("L-SERVO-FOOT-ROLL");

  auto RfootFrameIndex = morphLynX->getFrameIdxByName("R-FOOT");
  auto LfootFrameIndex = morphLynX->getFrameIdxByName("L-FOOT");


  Eigen::Vector3d RLegGRF, RLegGRT, LLegGRF, LLegGRT, footForce, footTorque, LLeg_COP, RLeg_COP, baseLinearVelocity, baseLinearVelocity_, baseLinearAcceleration;
  raisim::Vec<3> footPosition, footLinearVelocity, footAngularVelocity;
  raisim::Mat<3, 3> footOrientation;
  Affine3d Tir, Til, TiLH, TiRH, TiH;
  Eigen::Vector3d RfootPosition, LfootPosition, LfootLinearVelocity, LfootLinearVelocity_, LfootAngularVelocity, LfootLinearAcceleration,  RfootLinearVelocity, RfootLinearVelocity_, RfootAngularVelocity, RfootLinearAcceleration;
  Eigen::Matrix3d RfootOrientation, LfootOrientation;
  Quaterniond qir, qil;
  Vector3d COPL, COPR, ZMP, CoM_pos, CoM_vel;
  baseLinearVelocity.setZero();
  RfootLinearVelocity.setZero();
  LfootLinearVelocity.setZero();
  bool initialized = false;

  ROS_INFO("Initializing ROS publishers.."); 
  // INIT ROS PUBLISHER 
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/morphLynX_raisim_ros/joint_states", 1000);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/morphLynX_raisim_ros/odom", 1000);
  ros::Publisher wrench_pub_LLeg = n.advertise<geometry_msgs::WrenchStamped>("/morphLynX_raisim_ros/LLeg/force_torque_states", 1000);
  ros::Publisher wrench_pub_RLeg = n.advertise<geometry_msgs::WrenchStamped>("/morphLynX_raisim_ros/RLeg/force_torque_states", 1000);
  ros::Publisher gait_phase_pub = n.advertise<std_msgs::String>("/morphLynX_raisim_ros/gait_phase", 1000);
  ros::Publisher com_pub = n.advertise<nav_msgs::Odometry>("/morphLynX_raisim_ros/CoM", 1000);
  ros::Publisher odom_pub_LLeg = n.advertise<nav_msgs::Odometry>("/morphLynX_raisim_ros/LLeg/odom", 1000);
  ros::Publisher odom_pub_RLeg = n.advertise<nav_msgs::Odometry>("/morphLynX_raisim_ros/RLeg/odom", 1000);
  ros::Publisher zmp_pub = n.advertise<geometry_msgs::PointStamped>("/morphLynX_raisim_ros/ZMP", 1000);
  ros::Publisher LCOP_pub = n.advertise<geometry_msgs::PointStamped>("/morphLynX_raisim_ros/LLeg/COP", 1000);
  ros::Publisher RCOP_pub = n.advertise<geometry_msgs::PointStamped>("/morphLynX_raisim_ros/RLeg/COP", 1000);
  ros::Subscriber command_js_sub = n.subscribe("/morphLynX_raisim_ros/command_joint_states", 1000, commandJointStatesCallback);
  
  ros::Publisher imu_pub_LLeg = n.advertise<sensor_msgs::Imu>("/morphLynX_raisim_ros/LLeg/imu", 1000);
  ros::Publisher imu_pub_RLeg = n.advertise<sensor_msgs::Imu>("/morphLynX_raisim_ros/RLeg/imu", 1000);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/morphLynX_raisim_ros/imu", 1000);

  sensor_msgs::JointState joint_msg;
  joint_msg.name.resize(jnames.size());
  joint_msg.name = jnames;
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "world";
  odom_msg.child_frame_id = "base_link";
  nav_msgs::Odometry rodom_msg;
  rodom_msg.header.frame_id = "world";
  rodom_msg.child_frame_id = "r_sole";
  nav_msgs::Odometry lodom_msg;
  lodom_msg.header.frame_id = "world";
  lodom_msg.child_frame_id = "l_sole";

  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.frame_id = "world";
  std_msgs::String gait_msg;
  nav_msgs::Odometry com_msg;
  com_msg.header.frame_id = "world";
  com_msg.child_frame_id = "CoM";
  geometry_msgs::PointStamped zmp_msg;
  zmp_msg.header.frame_id = "world";
  geometry_msgs::PointStamped lcop_msg, rcop_msg;
  lcop_msg.header.frame_id = "world";
  rcop_msg.header.frame_id = "world";
  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = "world";


  ros::Rate loop_rate(freq);
  ROS_INFO("Entering main loop .."); 

  while (ros::ok())
  {

    morphLynX->getState(q, dq);
    /// Compute Base Linear Acceleration
    baseLinearVelocity_ = baseLinearVelocity;
    baseLinearVelocity = dq.head(3);
    baseLinearAcceleration = (baseLinearVelocity - baseLinearVelocity_) * freq;
    /// Get Leg Positions in raisim
    morphLynX->getFramePosition(RfootFrameIndex, footPosition);
    morphLynX->getFrameOrientation(RfootFrameIndex, footOrientation);
    RfootPosition = Eigen::Vector3d(footPosition[0], footPosition[1], footPosition[2]);
    //RfootOrientation << footOrientation[0], footOrientation[1], footOrientation[2], footOrientation[3], footOrientation[4], footOrientation[5], footOrientation[6], footOrientation[7], footOrientation[8];
    RfootOrientation = footOrientation.e();
    Tir.translation() = RfootPosition;
    Tir.linear() = RfootOrientation;
    qir = Quaterniond(RfootOrientation);
    morphLynX->getFrameVelocity(RfootFrameIndex, footLinearVelocity);
    RfootLinearVelocity_ = RfootLinearVelocity;
    RfootLinearVelocity = Eigen::Vector3d(footLinearVelocity[0], footLinearVelocity[1], footLinearVelocity[2]);
    RfootLinearAcceleration = (RfootLinearVelocity - RfootLinearVelocity_)*freq;
    morphLynX->getFrameAngularVelocity(RfootFrameIndex, footAngularVelocity);
    RfootAngularVelocity = Eigen::Vector3d(footAngularVelocity[0], footAngularVelocity[1], footAngularVelocity[2]);




    morphLynX->getFramePosition(LfootFrameIndex, footPosition);
    morphLynX->getFrameOrientation(LfootFrameIndex, footOrientation);
    LfootPosition = Eigen::Vector3d(footPosition[0], footPosition[1], footPosition[2]);
    //LfootOrientation << footOrientation[0], footOrientation[1], footOrientation[2], footOrientation[3], footOrientation[4], footOrientation[5], footOrientation[6], footOrientation[7], footOrientation[8];
    LfootOrientation = footOrientation.e();
    Til.translation() = LfootPosition;
    Til.linear() = LfootOrientation;
    qil = Quaterniond(LfootOrientation);
    morphLynX->getFrameVelocity(LfootFrameIndex, footLinearVelocity);
    LfootLinearVelocity_ = LfootLinearVelocity;
    LfootLinearVelocity = Eigen::Vector3d(footLinearVelocity[0], footLinearVelocity[1], footLinearVelocity[2]);
    LfootLinearAcceleration = (LfootLinearVelocity - LfootLinearVelocity_)*freq;

    morphLynX->getFrameAngularVelocity(LfootFrameIndex, footAngularVelocity);
    LfootAngularVelocity = Eigen::Vector3d(footAngularVelocity[0], footAngularVelocity[1], footAngularVelocity[2]);



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
    for (auto &contact : morphLynX->getContacts())
    {
      if (contact.skip())
        continue; /// if the contact is internal, one contact point is set to 'skip'
      if (RfootIndex == contact.getlocalBodyIndex())
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
      if (LfootIndex == contact.getlocalBodyIndex())
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
    {
      DS = true;
      LSS = false;
      RSS = false;
    }

    if (LLegGRF(2) > 1)
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
    if (RLegGRF(2) > 1)
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
    if(RLegGRF(2) > 1 || LLegGRF(2) > 1)
    {
      ZMP = (COPR*RLegGRF(2) + COPL*LLegGRF(2))/(LLegGRF(2) + RLegGRF(2));
      ZMP(2) = (COPR(2) + COPL(2))/2.0;
    }
    else
      ZMP.setZero();
      
    //cout<<"COP L "<<COPL.transpose()<<" "<<LLeg_COP.transpose()<<endl;
    //cout<<"COP R "<<COPR.transpose()<<" "<<RLeg_COP.transpose()<<endl;
    raisim::Vec<3> linear_momentum = morphLynX->getLinearMomentum();
    raisim::Vec<3> center_of_mass = morphLynX->getCOM();



    CoM_vel = Vector3d(linear_momentum(0), linear_momentum(1), linear_momentum(2)) / mass;
    CoM_pos = Vector3d(center_of_mass(0), center_of_mass(1), center_of_mass(2));

    //cout << "Buffer Size " << joint_data.size() << endl;

    if (joint_data.size() > 0)
    {
      sensor_msgs::JointStateConstPtr msg = joint_data.pop();
      std::vector<double> pos_vector = msg->position;
      double *pos_array = pos_vector.data();
      jointNominalConfig = Eigen::Map<Eigen::Matrix<double, 23, 1>>(pos_array);

      std::vector<double> vel_vector = msg->velocity;
      double *vel_array = vel_vector.data();
      jointNominalVelocity = Eigen::Map<Eigen::Matrix<double, 22, 1>>(vel_array);
    }
    if (animation_mode)
    {
      morphLynX->setGeneralizedCoordinate(jointNominalConfig);
      morphLynX->setGeneralizedVelocity(jointNominalVelocity);
    }
    else
    {
      morphLynX->setPdTarget(jointNominalConfig, jointNominalVelocity);
      VectorXd tau;
      // tau.setZero(morphLynX->getDOF());
      // tau.tail(morphLynX->getDOF()-6) = PDControl(q.tail(morphLynX->getGeneralizedCoordinateDim()-7), jointNominalConfig.tail(morphLynX->getGeneralizedCoordinateDim()-7), 
      // dq.tail(morphLynX->getDOF()-6), jointNominalVelocity.tail(morphLynX->getDOF()-6), jointPgain.tail(morphLynX->getDOF()-6).asDiagonal(), jointDgain.tail(morphLynX->getDOF()-6).asDiagonal());
      // cout<<"Joint Torque "<<tau.transpose()<<" "<<tau.size()<<endl;
      // morphLynX->setGeneralizedForce(tau);
    }


    if(!initialized)
    {
      initialized = true;
    }
    else
    {
      ///////////////////////////////// PUBLISHING ///////////////////
      // JOINT PUBLISHER
      // from Eigen:vectorXd to pub msg
      std::vector<double> qfoo(q.data() + 7, q.data() + q.size()); // convert  e to std vector
      joint_msg.position.resize(qfoo.size());
      joint_msg.position = qfoo;
      std::vector<double> dqfoo(dq.data() + 6, dq.data() + dq.size()); // convert  e to std vector
      joint_msg.velocity.resize(dqfoo.size());
      joint_msg.velocity = dqfoo;
      joint_msg.header.stamp = ros::Time::now();
      joint_pub.publish(joint_msg);

      // ODOM PUBLISHER
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
      odom_msg.header.stamp = ros::Time::now();
      odom_pub.publish(odom_msg);

      //RLeg Odom Publisher
      rodom_msg.pose.pose.position.x = RfootPosition(0);
      rodom_msg.pose.pose.position.y = RfootPosition(1);
      rodom_msg.pose.pose.position.z = RfootPosition(2);
      rodom_msg.pose.pose.orientation.x = qir.x();
      rodom_msg.pose.pose.orientation.y = qir.y();
      rodom_msg.pose.pose.orientation.z = qir.z();
      rodom_msg.pose.pose.orientation.w = qir.w();
      rodom_msg.twist.twist.linear.x = RfootLinearVelocity(0);
      rodom_msg.twist.twist.linear.y = RfootLinearVelocity(1);
      rodom_msg.twist.twist.linear.z = RfootLinearVelocity(2);
      rodom_msg.twist.twist.angular.x = RfootAngularVelocity(0);
      rodom_msg.twist.twist.angular.y = RfootAngularVelocity(1);
      rodom_msg.twist.twist.angular.z = RfootAngularVelocity(2);


      rodom_msg.header.stamp = ros::Time::now();
      odom_pub_RLeg.publish(rodom_msg);

      //LLeg Odom Publisher
      lodom_msg.pose.pose.position.x = LfootPosition(0);
      lodom_msg.pose.pose.position.y = LfootPosition(1);
      lodom_msg.pose.pose.position.z = LfootPosition(2);
      lodom_msg.pose.pose.orientation.x = qil.x();
      lodom_msg.pose.pose.orientation.y = qil.y();
      lodom_msg.pose.pose.orientation.z = qil.z();
      lodom_msg.pose.pose.orientation.w = qil.w();
      lodom_msg.twist.twist.linear.x = LfootLinearVelocity(0);
      lodom_msg.twist.twist.linear.y = LfootLinearVelocity(1);
      lodom_msg.twist.twist.linear.z = LfootLinearVelocity(2);
      lodom_msg.twist.twist.angular.x = LfootAngularVelocity(0);
      lodom_msg.twist.twist.angular.y = LfootAngularVelocity(1);
      lodom_msg.twist.twist.angular.z = LfootAngularVelocity(2);
      lodom_msg.header.stamp = ros::Time::now();
      odom_pub_LLeg.publish(lodom_msg);


      // FORCE TORQUE PUBLISHER
      wrench_msg.wrench.force.x = LLegGRF(0);
      wrench_msg.wrench.force.y = LLegGRF(1);
      wrench_msg.wrench.force.z = LLegGRF(2);
      wrench_msg.wrench.torque.x = LLegGRT(0);
      wrench_msg.wrench.torque.y = LLegGRT(1);
      wrench_msg.wrench.torque.z = LLegGRT(2);
      wrench_msg.header.stamp = ros::Time::now();
      wrench_pub_LLeg.publish(wrench_msg);
      wrench_msg.wrench.force.x = RLegGRF(0);
      wrench_msg.wrench.force.y = RLegGRF(1);
      wrench_msg.wrench.force.z = RLegGRF(2);
      wrench_msg.wrench.torque.x = RLegGRT(0);
      wrench_msg.wrench.torque.y = RLegGRT(1);
      wrench_msg.wrench.torque.z = RLegGRT(2);
      wrench_msg.header.stamp = ros::Time::now();
      wrench_pub_RLeg.publish(wrench_msg);

      // GAIT PHASE PUBLISHER
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
      com_msg.pose.pose.position.x = CoM_pos(0);
      com_msg.pose.pose.position.y = CoM_pos(1);
      com_msg.pose.pose.position.z = CoM_pos(2);
      com_msg.twist.twist.linear.x = CoM_vel(0);
      com_msg.twist.twist.linear.y = CoM_vel(1);
      com_msg.twist.twist.linear.z = CoM_vel(2);
      com_msg.header.stamp = ros::Time::now();
      com_pub.publish(com_msg);

      // ZERO MOMENT POINT PUBLISHER
      zmp_msg.point.x = ZMP(0);
      zmp_msg.point.y = ZMP(1);
      zmp_msg.point.z = ZMP(2);
      zmp_msg.header.stamp = ros::Time::now();
      zmp_pub.publish(zmp_msg);

      // COP PUBLISHER
      lcop_msg.point.x = COPL(0);
      lcop_msg.point.y = COPL(1);
      lcop_msg.point.z = COPL(2);
      lcop_msg.header.stamp = ros::Time::now();
      LCOP_pub.publish(lcop_msg);
      rcop_msg.point.x = COPR(0);
      rcop_msg.point.y = COPR(1);
      rcop_msg.point.z = COPR(2);
      rcop_msg.header.stamp = ros::Time::now();
      RCOP_pub.publish(rcop_msg);


      //IMU PUBLISHER
      imu_msg.angular_velocity.x = LfootAngularVelocity(0);
      imu_msg.angular_velocity.y = LfootAngularVelocity(1);
      imu_msg.angular_velocity.z = LfootAngularVelocity(2);
      imu_msg.linear_acceleration.x = LfootLinearAcceleration(0);
      imu_msg.linear_acceleration.y = LfootLinearAcceleration(1);
      imu_msg.linear_acceleration.z = LfootLinearAcceleration(2);
      imu_msg.orientation.x = qil.x();
      imu_msg.orientation.y = qil.y();
      imu_msg.orientation.z = qil.z();
      imu_msg.orientation.w = qil.w();
      imu_msg.header.stamp = ros::Time::now();
      imu_pub_LLeg.publish(imu_msg);

      imu_msg.angular_velocity.x = RfootAngularVelocity(0);
      imu_msg.angular_velocity.y = RfootAngularVelocity(1);
      imu_msg.angular_velocity.z = RfootAngularVelocity(2);
      imu_msg.linear_acceleration.x = RfootLinearAcceleration(0);
      imu_msg.linear_acceleration.y = RfootLinearAcceleration(1);
      imu_msg.linear_acceleration.z = RfootLinearAcceleration(2);
      imu_msg.orientation.x = qir.x();
      imu_msg.orientation.y = qir.y();
      imu_msg.orientation.z = qir.z();
      imu_msg.orientation.w = qir.w();
      imu_msg.header.stamp = ros::Time::now();
      imu_pub_RLeg.publish(imu_msg);


      imu_msg.angular_velocity.x = dq(3);
      imu_msg.angular_velocity.y = dq(4);
      imu_msg.angular_velocity.z = dq(5);
      imu_msg.linear_acceleration.x = baseLinearAcceleration(0);
      imu_msg.linear_acceleration.y = baseLinearAcceleration(1);
      imu_msg.linear_acceleration.z = baseLinearAcceleration(2);
      imu_msg.orientation.x = q(4);
      imu_msg.orientation.y = q(5);
      imu_msg.orientation.z = q(6);
      imu_msg.orientation.w = q(3);
      imu_msg.header.stamp = ros::Time::now();
      imu_pub.publish(imu_msg);

    }
    ////////////////////////////////// INTERGRATE ///////////////////////
    loop_rate.sleep();
    server.integrateWorldThreadSafe();
    ros::spinOnce();
  }

  server.killServer();
}
