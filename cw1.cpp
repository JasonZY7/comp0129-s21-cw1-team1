/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

 /* Student(s): Zinan Liu, Zhiyuan Song, Chi Zhang */

#include <cw1.h>

CW1::CW1 (ros::NodeHandle &nh):
  debug_ (false)
{
  nh_ = nh;
  initParams ();
  updateParams (nh);

  object_pose_sub_ = nh.subscribe("/object_pose",10,&CW1::objPoseCallback, this );
  Touch_pub_ = nh.advertise<std_msgs::Bool>("/bumper2_touched", 10);
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::initParams ()
{
  // Frames
  this->world_frame_ = "/world_frame";
  
  // Topics
  this->robot_frame_ = "/robot_frame";
  
  // HW1-Q2: setup the names for frames {0}, {1}, {2}, and {3}
  this->frame_0_ = "/frame_0";
  this->frame_1_ = "/frame_1";
  this->frame_2_ = "/frame_2";
  this->frame_3_ = "/frame_3";
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::updateParams (ros::NodeHandle &nh)
{
  // Frames
  nh.getParam("/frame_params/world_frame", this->world_frame_);
  nh.getParam("/frame_params/robot_frame", this->robot_frame_);
  nh.getParam("/frame_params/frame_0", this->frame_0_);
  nh.getParam("/frame_params/frame_1", this->frame_1_);
  nh.getParam("/frame_params/frame_2", this->frame_2_);
  nh.getParam("/frame_params/frame_3", this->frame_3_);
}

void 
CW1::objPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  object_pose = msg;
}

////////////////////////////////////////////////////////////////////////////////
std::string
CW1::getWorldFrame ()
{
  return (this->world_frame_);
}

////////////////////////////////////////////////////////////////////////////////
std::string
CW1::getRobotFrame ()
{
  return (this->robot_frame_);
}

////////////////////////////////////////////////////////////////////////////////

int
CW1::kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;
  
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
  ch = getchar();
  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::Lab1CreateFrames ()
{
  // generate a robot frame attached to the world frame (0,0,0)
  transf_.setOrigin (tf::Vector3(0.0, 0.0, 3.0));
  transf_.setRotation (tf::Quaternion(0.0, 0.0, 0.0, 1.0)); //quaternion
  
  // Note that the rotation can be given in various forms (rotation matrix,
  // axis-angle, etc), but the function setRotation gets a tf::Quaternion,
  // thus when a different type rotation is available, it should be converted
  // to a tf::Quaternion.
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::Lab1PublishFrames ()
{
  // publish world->robot
  tranf_br_.sendTransform(tf::StampedTransform(transf_,
                                               ros::Time::now(), 
                                               world_frame_,
                                               robot_frame_));
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q1GenFrames ()
{  
  // ToDo: create transformations

  // generate a frame1 attached to the world frame (0,0,0)
  transf_01_.setOrigin (tf::Vector3(0.0, 1.0, 1.0));
  transf_01_.setRotation (tf::Quaternion(0.0, 0.0, 0.0, 1.0)); //quaternion

  // generate a frame2 attached to the world frame (0,0,0)
  transf_02_.setOrigin (tf::Vector3(-0.5, 1.5, 1.1));
  transf_02_.setRotation (tf::Quaternion(0.0, 0.0, 0.0, 1.0)); //quaternion

  // generate a frame3 attached to the world frame (0,0,0)
  transf_03_.setOrigin (tf::Vector3(-0.5, 1.5, 3.0));
  transf_03_.setRotation (tf::Quaternion(0.7071068, 0.7071068, 0.0, 0.0)); //quaternion

  return;
}

////////////////////////////////////////////////////////////////////////////////
bool print_once = true;
void
CW1::cw1Q1PubFrames ()
{
  ros::Rate rate(100);
  int k=0;
  double x,y,z,w,dx,dy,dz;
  Eigen::Matrix4d T23;
  tf2_ros::Buffer tfBuffer;
  tf2_ros:: TransformListener tfListener(tfBuffer);

  while (nh_.ok() && k<100){
    cw1Q1GenFrames();
    transf_01_br_.sendTransform(tf::StampedTransform(transf_01_, 
                                ros::Time::now(), "world", frame_1_));
    transf_02_br_.sendTransform(tf::StampedTransform(transf_02_, 
                                ros::Time::now(), "world", frame_2_));
    transf_03_br_.sendTransform(tf::StampedTransform(transf_03_,
                                ros::Time::now(), "world", frame_3_));
    if(print_once){
      geometry_msgs::TransformStamped T;
      try
      {
        T = tfBuffer.lookupTransform("frame_2", "frame_3",
                                    ros::Time(0),ros::Duration(3.0));
      } catch (tf2::TransformException &exception)
      {
        ROS_WARN ("%s", exception.what ());
        continue;
      }

      x = T.transform.rotation.x,
      y = T.transform.rotation.y,
      z = T.transform.rotation.z,
      w = T.transform.rotation.w;

      dx = T.transform.translation.x,
      dy = T.transform.translation.y,
      dz = T.transform.translation.z;

      T23(0,0) = 1-2*y*y-2*z*z;
      T23(0,1) = 2*x*y-2*z*w;
      T23(0,2) = 2*x*z+2*y*w;
      T23(0,3) = dx;

      T23(1,0) = 2*x*y+2*z*w;
      T23(1,1) = 1-2*x*x-2*z*z;
      T23(1,2) = 2*y*z-2*x*w;
      T23(1,3) = dy;

      T23(2,0) = 2*x*z-2*y*w;
      T23(2,1) = 2*y*z+2*x*w;
      T23(2,2) = 1-2*x*x-2*y*y;
      T23(2,3) = dz;

      T23(3,0) = 0;
      T23(3,1) = 0;
      T23(3,2) = 0;
      T23(3,3) = 1;
      
      //zero the small number
      for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
          // ~e-16 can be zeroed, set threshould 1e-15 
          if(abs(T23(i,j))<1e-15){
            T23(i,j) = 0;
          }
        }
      }

      ROS_INFO_STREAM("2_T_3:\n" << T23);
      print_once = false;
    }
    rate.sleep();
    k++;
  }
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2HomeRobot (ros::Publisher &joint_pub, int i)
{
  // ToDo: setup and publish the joint_state
  std::string name[] = {"panda_joint1", "panda_joint2", "panda_joint3",
                        "panda_joint4", "panda_joint5", "panda_joint6",
                        "panda_joint7", "panda_finger_joint1"};
  float panda_home_state[] = {0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.785398, 0.0};
  sensor_msgs::JointState joint;
  joint.name.push_back(name[i]);
  joint.position.push_back(panda_home_state[i]);
  joint_pub.publish(joint);
}


////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2PrintTFs ()
{
  // ToDo: lookup the transformation and print it
  geometry_msgs::TransformStamped T0e;
  geometry_msgs::TransformStamped T2e;
  tf2_ros::Buffer tfBuffer;
  tf2_ros:: TransformListener tfListener(tfBuffer);
  Eigen::Matrix4d T_0e;
  Eigen::Matrix4d T_2e;
  double x0e,y0e,z0e,w0e,dx0e,dy0e,dz0e;
  double x2e,y2e,z2e,w2e,dx2e,dy2e,dz2e;

  try
  {
    T0e = tfBuffer.lookupTransform("panda_link0", "panda_hand",
                                  ros::Time(0),ros::Duration(3.0));
    T2e = tfBuffer.lookupTransform("panda_link2", "panda_hand",
                                  ros::Time(0),ros::Duration(3.0));
  } catch (tf2::TransformException &exception)
  {
    ROS_WARN ("%s", exception.what ());
  }
  
  //T0e transform
  x0e = T0e.transform.rotation.x,
  y0e = T0e.transform.rotation.y,
  z0e = T0e.transform.rotation.z,
  w0e = T0e.transform.rotation.w;

  dx0e = T0e.transform.translation.x,
  dy0e = T0e.transform.translation.y,
  dz0e = T0e.transform.translation.z;

  T_0e(0,0) = 1-2*y0e*y0e-2*z0e*z0e;
  T_0e(0,1) = 2*x0e*y0e-2*z0e*w0e;
  T_0e(0,2) = 2*x0e*z0e+2*y0e*w0e;
  T_0e(0,3) = dx0e;

  T_0e(1,0) = 2*x0e*y0e+2*z0e*w0e;
  T_0e(1,1) = 1-2*x0e*x0e-2*z0e*z0e;
  T_0e(1,2) = 2*y0e*z0e-2*x0e*w0e;
  T_0e(1,3) = dy0e;

  T_0e(2,0) = 2*x0e*z0e-2*y0e*w0e;
  T_0e(2,1) = 2*y0e*z0e+2*x0e*w0e;
  T_0e(2,2) = 1-2*x0e*x0e-2*y0e*y0e;
  T_0e(2,3) = dz0e;

  T_0e(3,0) = 0;
  T_0e(3,1) = 0;
  T_0e(3,2) = 0;
  T_0e(3,3) = 1;

  //T2e transform
  x2e = T2e.transform.rotation.x,
  y2e = T2e.transform.rotation.y,
  z2e = T2e.transform.rotation.z,
  w2e = T2e.transform.rotation.w;

  dx2e = T2e.transform.translation.x,
  dy2e = T2e.transform.translation.y,
  dz2e = T2e.transform.translation.z;

  T_2e(0,0) = 1-2*y2e*y2e-2*z2e*z2e;
  T_2e(0,1) = 2*x2e*y2e-2*z2e*w2e;
  T_2e(0,2) = 2*x2e*z2e+2*y2e*w2e;
  T_2e(0,3) = dx2e;

  T_2e(1,0) = 2*x2e*y2e+2*z2e*w2e;
  T_2e(1,1) = 1-2*x2e*x2e-2*z2e*z2e;
  T_2e(1,2) = 2*y2e*z2e-2*x2e*w2e;
  T_2e(1,3) = dy2e;

  T_2e(2,0) = 2*x2e*z2e-2*y2e*w2e;
  T_2e(2,1) = 2*y2e*z2e+2*x2e*w2e;
  T_2e(2,2) = 1-2*x2e*x2e-2*y2e*y2e;
  T_2e(2,3) = dz2e;

  T_2e(3,0) = 0;
  T_2e(3,1) = 0;
  T_2e(3,2) = 0;
  T_2e(3,3) = 1;

  // zero the small number
  for (int i=0; i<4; i++){
    for (int j=0; j<4; j++){
      if(abs(T_0e(i,j))<1e-3){
        T_0e(i,j) = 0;
      }
      if(abs(T_2e(i,j))<1e-3){
        T_2e(i,j) = 0;
      }
    }
  }

  //print transform matrix
  ROS_INFO_STREAM("0_T_e:\n" << T_0e);
  ROS_INFO_STREAM("2_T_e:\n" << T_2e);
}

void
CW1::cw1Q2RotJoint (ros::Publisher &joint_pub)
{
  // ToDo: setup the joint_state
  // ToDo: setup and publish the joint_state
  std::string name = "panda_joint2";
  float joint2_k = 1.5707;
  sensor_msgs::JointState joint;
  joint.name.push_back(name);
  joint.position.push_back(joint2_k);
  joint_pub.publish(joint);
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2PubHandPose (ros::Publisher &ee_pose_pub)
{
  // ToDo: create and publish the end effector pose message
  geometry_msgs::TransformStamped T0e;
  tf2_ros::Buffer tfBuffer;
  tf2_ros:: TransformListener tfListener(tfBuffer);
  geometry_msgs::PoseStamped T_0e;
  try
  {
    T0e = tfBuffer.lookupTransform("panda_link0", "panda_hand",
                                  ros::Time(0),ros::Duration(3.0));
  } catch (tf2::TransformException &exception)
  {
    ROS_WARN ("%s", exception.what ());
    // continue;
  }
  T_0e.pose.position.x = T0e.transform.translation.x;
  T_0e.pose.position.y = T0e.transform.translation.y;
  T_0e.pose.position.z = T0e.transform.translation.z;

  T_0e.pose.orientation.x = T0e.transform.rotation.x;
  T_0e.pose.orientation.y = T0e.transform.rotation.y;
  T_0e.pose.orientation.z = T0e.transform.rotation.z;
  T_0e.pose.orientation.w = T0e.transform.rotation.w;
  
  ee_pose_pub.publish(T_0e);
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface& 
                       planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type =
    collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table: center of the cube. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type =
    collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table: center of the cube. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type =
    collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object: center of the cylinder. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3PubObjPose (ros::Publisher &obj_pub)
{ 
  //check distance between left and right finger
  geometry_msgs::TransformStamped T0_lf, T0_rf, T0e;
  geometry_msgs::PoseStamped T_obj_0;
  tf2_ros::Buffer tfBuffer;
  tf2_ros:: TransformListener tfListener(tfBuffer);
  Eigen::Matrix4d T_e0;
  Eigen::Vector3d Obj_pos_0(0.5,0,0.5);
  double xlf,ylf,zlf;
  double xrf,yrf,zrf;
  double x0e,y0e,z0e,w0e,dx0e,dy0e,dz0e;
  double dist_f, dis;
  try
  {
    T0_lf = tfBuffer.lookupTransform("panda_link0", "panda_leftfinger",
                                    ros::Time(0),ros::Duration(3.0));
    T0_rf = tfBuffer.lookupTransform("panda_link0", "panda_rightfinger",
                                    ros::Time(0),ros::Duration(3.0));
    T0e = tfBuffer.lookupTransform("panda_link0", "panda_hand",
                                    ros::Time(0),ros::Duration(3.0));
  } catch (tf2::TransformException &exception)
  {
    ROS_WARN ("%s", exception.what ());
  }
  xlf = T0_lf.transform.translation.x;
  ylf = T0_lf.transform.translation.y;
  zlf = T0_lf.transform.translation.z;
  
  xrf = T0_rf.transform.translation.x;
  yrf = T0_rf.transform.translation.y;
  zrf = T0_rf.transform.translation.z;

  //T0e matrix transform
  x0e = T0e.transform.rotation.x,
  y0e = T0e.transform.rotation.y,
  z0e = T0e.transform.rotation.z,
  w0e = T0e.transform.rotation.w;

  dx0e = T0e.transform.translation.x,
  dy0e = T0e.transform.translation.y,
  dz0e = T0e.transform.translation.z;

  // euclidean distance between finger
  dist_f = sqrt(pow(xlf-xrf,2)+pow(ylf-yrf,2)+pow(zlf-zrf,2));
  // euclidean distance between hand and centre of object at table 2
  dis = sqrt(pow(dx0e-0,2)+pow(dy0e-0.5,2)+pow(dz0e-0.5,2));
  // tolerance defined
  double tol = 1e-5;
  if (dist_f <= (0+tol)) {
    // translation at centre of table1 when grasped
    T_obj_0.pose.position.x = dx0e;
    T_obj_0.pose.position.y = dy0e;
    T_obj_0.pose.position.z = dz0e;
    // orientatioin at centre of table1 when grasped
    T_obj_0.pose.orientation.x = x0e;
    T_obj_0.pose.orientation.y = y0e;
    T_obj_0.pose.orientation.z = z0e;
    T_obj_0.pose.orientation.w = w0e;
  }
  else if (dis < 0.35 && dist_f >= (0+tol)){
    // translation at centre of table1 when release
    T_obj_0.pose.position.x = 0;
    T_obj_0.pose.position.y = 0.5;
    T_obj_0.pose.position.z = 0.5;
    // orientatioin at centre of table1 when release
    T_obj_0.pose.orientation.x = x0e;
    T_obj_0.pose.orientation.y = y0e;
    T_obj_0.pose.orientation.z = z0e;
    T_obj_0.pose.orientation.w = w0e;
  }
  else{
    // translation at table2 when ungrasped
    T_obj_0.pose.position.x = Obj_pos_0[0];
    T_obj_0.pose.position.y = Obj_pos_0[1];
    T_obj_0.pose.position.z = Obj_pos_0[2];
    // orientatioin at table2 when ungrasped
    T_obj_0.pose.orientation.x = 0.70768;
    T_obj_0.pose.orientation.y = 0.00350509;
    T_obj_0.pose.orientation.z = 0.706523;
    T_obj_0.pose.orientation.w = 0.00141894;
  }
  obj_pub.publish(T_obj_0);
  return;
}

////////////////////////////////////////////////////////////////////////////////
double dist;
void
CW1::cw1Q3PubEeObjClose (ros::Publisher &ee_obj_close_pub)
{
  // end effector pos computation
  geometry_msgs::TransformStamped T0e;
  tf2_ros::Buffer tfBuffer;
  tf2_ros:: TransformListener tfListener(tfBuffer);
  std_msgs::Bool msg;
  double x,y,z;
  double xo,yo,zo;
  try
  {
    T0e = tfBuffer.lookupTransform("panda_link0", "panda_hand",
                                  ros::Time(0),ros::Duration(3.0));
  } catch (tf2::TransformException &exception)
  {
    ROS_WARN ("%s", exception.what ());
    // continue;
  }
  x = T0e.transform.translation.x;
  y = T0e.transform.translation.y;
  z = T0e.transform.translation.z;

  // obj pos defined
  xo = 0.5;
  yo = 0;
  zo = 0.5;

  // euclidean distance compute
  dist = sqrt(pow(x-xo,2)+pow(y-yo,2)+pow(z-zo,2));

  // distance checking
  if (dist<0.2){
    msg.data = true;
  }
  else{
    msg.data = false;
  }
  ee_obj_close_pub.publish(msg);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
cw1Q3EeObjCloseCallBack(const std_msgs::Bool::ConstPtr& distmsg){
  bool msg = distmsg->data;
  if (msg) {
    ROS_INFO_STREAM("ee_obj_close:\tTRUE\nObject-Gripper Dist: " 
    << std::setprecision(2) << dist << "m");
  }
  else{
    ROS_INFO_STREAM("ee_obj_close:\tFALSE\nObject-Gripper Dist: " 
    << std::setprecision(2) << dist << "m");
  }
}

void
CW1::cw1Q3EeObjClosePrint (moveit::planning_interface::MoveGroupInterface& move_group)
{
  ros::Subscriber ee_obj_close_sub = nh_.subscribe("/ee_obj_close", 1, cw1Q3EeObjCloseCallBack);

  ros::spinOnce();
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void
CW1::cw1Q3ReachObj (moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.42;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  move_group.setSupportSurfaceName("table1");
  move_group.pick("object", grasps);
  return;
}
 
////////////////////////////////////////////////////////////////////////////////
bool s;
void
CW1::cw1Q3SelectSideGrasp (moveit::planning_interface::MoveGroupInterface& move_group)
{
  ROS_INFO_STREAM( "Strategy S is selected: \n ");
  s = true;
  return;
}

////////////////////////////////////////////////////////////////////////////////
bool t;
void
CW1::cw1Q3SelectTopGrasp (moveit::planning_interface::MoveGroupInterface& move_group)
{
  ROS_INFO_STREAM( "Strategy T is selected: \n ");
  t = true;
  return;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<moveit_msgs::Grasp> grasps;
void
CW1::cw1Q3GraspObj (moveit::planning_interface::MoveGroupInterface& move_group)
{
  boost::shared_ptr<std_msgs::Bool const> ee_obj_close_msg;
  ee_obj_close_msg = 
    ros::topic::waitForMessage<std_msgs::Bool>("/ee_obj_close", ros::Duration(1));
  std_msgs::Bool msg = *ee_obj_close_msg;

  geometry_msgs::TransformStamped T0e;
  tf2_ros::Buffer tfBuffer;
  tf2_ros:: TransformListener tfListener(tfBuffer);
  double x,y,z;
  double xt,yt,zt;
  bool grap;

  if (s){
      grasps.resize(1);
      grasps[0].grasp_pose.header.frame_id = "panda_link0";
      tf2::Quaternion orientation;
      orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
      grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
      grasps[0].grasp_pose.pose.position.x = 0.415;
      grasps[0].grasp_pose.pose.position.y = 0;
      grasps[0].grasp_pose.pose.position.z = 0.5;

      grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
      grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
      grasps[0].pre_grasp_approach.min_distance = 0.095;
      grasps[0].pre_grasp_approach.desired_distance = 0.115;

      grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
      grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
      grasps[0].post_grasp_retreat.min_distance = 0.1;
      grasps[0].post_grasp_retreat.desired_distance = 0.25;
      if (msg.data){
        openGripper(grasps[0].pre_grasp_posture);
        closedGripper(grasps[0].grasp_posture);
      }
      move_group.setSupportSurfaceName("table1");
      move_group.pick("object", grasps);
  }

  else if(t){
    grasps.resize(1);
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0, -M_PI/4);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.5;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.685;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // euculidean distance between hand and top of object at pre grasp pos
    try
    {
      T0e = tfBuffer.lookupTransform("panda_link0", "panda_hand",
                                    ros::Time(0),ros::Duration(3.0));
    } catch (tf2::TransformException &exception)
    {
      ROS_WARN ("%s", exception.what ());
      // continue;
    }
    x = T0e.transform.translation.x;
    y = T0e.transform.translation.y;
    z = T0e.transform.translation.z;

    // obj top pos defined
    xt = 0.5;
    yt = 0;
    zt = 0.6;

    // euclidean distance compute
    dist = sqrt(pow(x-xt,2)+pow(y-yt,2)+pow(z-zt,2));
    if (dist < 0.2) {
      openGripper(grasps[0].pre_grasp_posture);
      closedGripper(grasps[0].grasp_posture);
    }
    move_group.setSupportSurfaceName("table1");
    move_group.pick("object", grasps);
  }
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3MoveObj (moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  openGripper(place_location[0].post_place_posture);

  move_group.setSupportSurfaceName("table2");
  move_group.place("object", place_location);
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::cw1Q3ObjTouch (moveit::planning_interface::MoveGroupInterface& move_group)
{
  std_msgs::Bool touch;
  if( object_pose.pose.position.x==0.0 && object_pose.pose.position.y==0.5 && object_pose.pose.position.z==0.5){
    touch.data = true;
    Touch_pub_.publish(touch);
  }
  else{
    touch.data = false;
    Touch_pub_.publish(touch);
  }
  return;
}
