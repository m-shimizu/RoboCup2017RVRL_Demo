/*  This file was reformed 
       by Masaru Shimizu
         for RoboCup 2017 Rescue Virtual Robot League
           at 23.July.2017  */
/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_diff_drive.h
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin 
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */

#ifndef SETJOINTANGLE_H
#define SETJOINTANGLE_H

#include <map>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

/*
// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
*/

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class setJointAngle : public ModelPlugin {

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
    public:
      setJointAngle();
      ~setJointAngle();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Reset();

    protected:
      virtual void UpdateChild(void);
//      virtual void FiniChild(void);

    private:
      void getParameter_int(int& dst, const char* tagName, int defaultValue);
      void getParameter_double(double& dst, const char* tagName, double defaultValue);
      void getParameter_string(std::string& dst, const char* tagName, const char* defaultValue);
      void PID_Control(void);
      void check_key_command(void);

//      GazeboRosPtr                   gazebo_ros_;
      physics::ModelPtr              parent;
      sdf::ElementPtr                sdf;
      transport::NodePtr             node;
      event::ConnectionPtr           update_connection_;

      int                            moveFlag, moveJoint;
      double                         moveAngleWidth;
      int                            maxAngles;
      double*                        targetAngles;
      std::vector<std::string>       angleNames;
      std::vector<physics::JointPtr> joints;

/*
      // ROS STUFF
      boost::mutex lock;
      ros::Subscriber cmd_arm12_subscriber_;
      ros::Subscriber cmd_hand12_subscriber_;

      std::string robot_namespace_;
      std::string command_topic1_;
      std::string command_topic2_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // Callback stuff
      void cmdarm12_Callback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
      void cmdhand12_Callback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      bool alive_;
*/

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;
            
    // Flags

  };

}

#endif

