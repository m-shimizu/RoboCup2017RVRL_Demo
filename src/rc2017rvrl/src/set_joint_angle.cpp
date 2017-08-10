/*  This file was reformed 
       by Masaru Shimizu
         for RoboCup 2017 Rescue Virtual Robot League
           at 23.July.2017  */

/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */


/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */

#include <algorithm>
#include <assert.h>

#include "set_joint_angle.h"

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

//#include <ros/ros.h>

#define D_maxAngles 30
#define maxLengthAngleName 30
const char* angleTagName[D_maxAngles]={"angleName0", 
                                 "angleName1",
                                 "angleName2",
                                 "angleName3",
                                 "angleName4",
                                 "angleName5",
                                 "angleName6",
                                 "angleName7",
                                 "angleName8",
                                 "angleName9",
                                "angleName10",
                                "angleName11",
                                "angleName12",
                                "angleName13",
                                "angleName14",
                                "angleName15",
                                "angleName16",
                                "angleName17",
                                "angleName18",
                                "angleName19",
                                "angleName20",
                                "angleName21",
                                "angleName22",
                                "angleName23",
                                "angleName24",
                                "angleName25",
                                "angleName26",
                                "angleName27",
                                "angleName28",
                                "angleName29" };


namespace gazebo
{

// Constructor
setJointAngle::setJointAngle(void) 
{
  targetAngles = NULL;
//  angleName   = (typeof angleName)NULL;
}

// Destructor
setJointAngle::~setJointAngle() 
{
  if(NULL!=targetAngles)
    delete targetAngles;
//  if((typeof angleName)NULL!=angleName)
//    delete angleName;
}

void setJointAngle::getParameter_int(int& dst, const char* tagName, int defaultValue)
{
  if(!sdf->HasElement(tagName))
    dst = defaultValue;
  else
    dst = sdf->GetElement(tagName)->Get<int>();
}

void setJointAngle::getParameter_double(double& dst, const char* tagName, double defaultValue)
{
  if(!sdf->HasElement(tagName))
    dst = defaultValue;
  else
    dst = sdf->GetElement(tagName)->Get<double>();
}

void setJointAngle::getParameter_string(std::string& dst, const char* tagName, const char* defaultValue)
{
  if(!sdf->HasElement(tagName))
    dst = defaultValue;
  else
    dst = sdf->GetElement(tagName)->Get<std::string>();
}

// Load the controller
void setJointAngle::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->parent = _parent;
  this->sdf    = _sdf;
  
/*
  gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "setJointAngle" ) );
  // Make sure the ROS node for Gazebo has already been initialized
  gazebo_ros_->isInitialized();
*/

  node = transport::NodePtr(new transport::Node());
  node->Init(this->parent->GetWorld()->GetName());
  
  getParameter_int(maxAngles, "maxAngles", 0);
  getParameter_int(moveFlag, "moveOn", 0);
  getParameter_int(moveJoint,"moveJoint", 0);
  getParameter_double(moveAngleWidth,"moveAngleWidth", 0);
  if(moveJoint >= maxAngles)
    moveFlag = 0;
//  gazebo_ros_->getParameter<int> (maxAngles, "maxAngles", 0);
  if(0==maxAngles)
  {
    printf("[ERR] setJointAngle : no maxAngles\n");
    return;
  }
  targetAngles = new double[maxAngles];
  angleNames.resize(maxAngles);
  joints.resize(maxAngles);
  for(int i=0; i < maxAngles; i++)
  {
    getParameter_string(angleNames[i], angleTagName[i], "");
    getParameter_double(targetAngles[i], angleNames[i].c_str(), (double)0.0 );
    joints[i] = parent->GetJoint(angleNames[i].c_str());
/*  
    gazebo_ros_->getParameter<std::string> ( angleNames[i], angleTagName[i], "" );
    gazebo_ros_->getParameter<double> ( targetAngles[i], angleNames[i].c_str(), (double)0.0 );
    joints[i] = gazebo_ros_->getJoint ( parent, angleTagName[i], "" );
*/
    if(0!=angleNames[i].size() && 0==joints[i])
      std::cout << "Error getting joint: " << angleNames[i] << "\n";
  }

  last_update_time_ = parent->GetWorld()->GetSimTime();

  // listen to the update event (broadcast every simulation iteration)
  this->update_connection_ =
    event::Events::ConnectWorldUpdateBegin ( boost::bind ( &setJointAngle::UpdateChild, this ) );
/*
  alive_ = true;

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic1_.c_str());
  ros::SubscribeOptions so1 =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic1_, 1,
        boost::bind(&setJointAngle::cmdarm12_Callback, this, _1),
        ros::VoidPtr(), &queue_);
  cmd_arm12_subscriber_ = gazebo_ros_->node()->subscribe(so1); // DO NOT REMOVE "cmd_arm12_subscriber_ = "
  ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), command_topic1_.c_str());

  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic2_.c_str());
  ros::SubscribeOptions so2 =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic2_, 1,
        boost::bind(&setJointAngle::cmdhand12_Callback, this, _1),
        ros::VoidPtr(), &queue_);
  cmd_hand12_subscriber_ = gazebo_ros_->node()->subscribe(so2); // DO NOT REMOVE "cmd_hand12_subscriber_ = "
  ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), command_topic2_.c_str());

  // start custom queue for diff drive
  this->callback_queue_thread_ =
    boost::thread ( boost::bind ( &setJointAngle::QueueThread, this ) );

  printf("\n#####################################################\n");
  printf("#####################################################\n");
  printf("Type the following command in another terminal.\n");
  printf("rostopic pub -r 10 /my_arm_robot/cmd_arm12 geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'\n");
  printf("#####################################################\n");
  printf("#####################################################\n");
*/
}

void setJointAngle::Reset()
{
  last_update_time_ = parent->GetWorld()->GetSimTime();
}

void setJointAngle::PID_Control(void)
{
  double angleMonitor, angleTarget, order, w = (2*M_PI/2);
  /*
  static int counter = 0;
  if(counter++ < 10)
    return;
  else
    counter = 0;
  */
  for(int i = 0 ; i < maxAngles; i++)
  {
    if(0==joints[i])
      continue;
    angleMonitor = joints[i]->GetAngle(0).Radian();
  // printf("Monitor Angle[%d] : %f\n", i, joints_[i]->GetAngle(0).Degree());

  // You can use velocity control. NO EFFORT CONTROL
    angleTarget = targetAngles[i] / 180 * M_PI;
    if(0!=moveFlag && moveJoint==i)
      angleTarget = moveAngleWidth / 180 * M_PI * sin(w * parent->GetWorld()->GetSimTime().Double());
    order = -2 * (angleMonitor - angleTarget);

    parent->GetJointController()->SetVelocityPID(
        joints[i]->GetScopedName(), common::PID(0.1, 0, 0));
    joints[i]->SetVelocity(0, order);
  }
}

#include <termios.h>
#include <fcntl.h>

// To know pushing any key
int  doslike_kbhit(void)
{
  struct termios  oldt, newt;
  int  ch;
  int  oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
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

/////////////////////////////////////////////////
// To gwt a charactor code of a pushed key
int  doslike_getch(void)
{
  static struct termios  oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

void  setJointAngle::check_key_command(void)
{
  if(doslike_kbhit())
  {
  int cmd = doslike_getch();
    switch(cmd)
    {
/*
      case 'q': Target_Angles_[FINGER1] += 0.05;
          break;
      case 'a': Target_Angles_[FINGER1] -= 0.05;
          break;
      case 'w': Target_Angles_[SHOULDERYAW] += 0.05;
          break;
      case 's': Target_Angles_[SHOULDERYAW] -= 0.05;
          break;
      case 'e': Target_Angles_[SHOULDERPITCH] += 0.05;
          break;
      case 'd': Target_Angles_[SHOULDERPITCH] -= 0.05;
          break;
*/
    }
  }
}

// Update the controller
void setJointAngle::UpdateChild()
{
//  check_key_command();
  PID_Control();
}

/*
// Finalize the controller
void setJointAngle::FiniChild()
{
  alive_ = false;
  queue_.clear();
  queue_.disable();
  gazebo_ros_->node()->shutdown();
  callback_queue_thread_.join();
}

void setJointAngle::cmdarm12_Callback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
  Target_Angles_[SHOULDERYAW]   = cmd_msg->linear.x;
  Target_Angles_[SHOULDERPITCH] = cmd_msg->linear.y;
  Target_Angles_[FINGER1]       = cmd_msg->linear.z;
  Target_Angles_[FINGER2]       = cmd_msg->angular.x;
  Target_Angles_[FINGER12]      = cmd_msg->angular.y;
  Target_Angles_[FINGER22]      = cmd_msg->angular.z;
//  printf("cmdarm12 : x,y,z,r,p,y=%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n"
//            , cmd_msg->linear.x, cmd_msg->linear.y, cmd_msg->linear.z,
//              cmd_msg->angular.x, cmd_msg->angular.y, cmd_msg->angular.z);
}

void setJointAngle::cmdhand12_Callback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
  printf("cmdhand12 : x,y,z,r,p,y=%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n"
            , cmd_msg->linear.x, cmd_msg->linear.y, cmd_msg->linear.z,
              cmd_msg->angular.x, cmd_msg->angular.y, cmd_msg->angular.z);
}

void setJointAngle::QueueThread()
{
  static const double timeout = 0.01;
  while ( alive_ && gazebo_ros_->node()->ok() )
  {
    queue_.callAvailable ( ros::WallDuration ( timeout ) );
  }
}
*/

GZ_REGISTER_MODEL_PLUGIN ( setJointAngle )
}

