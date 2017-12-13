/**********************************************************************************
Copyright (c) 2017, Kinova Robotics inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.
      
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************************/

#include <movo_assisted_teleop/movo_assisted_teleop.h>

static const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;


inline double modulus_fnc(double x, double y)
{
    if (0.0 == y)
    {
        return x;
    }
    
    double m= x - y * floor(x/y);

    // handle boundary cases resulted from floating-point cut off:

    if (y > 0.0)              // modulo range: [0..y)
    {
        if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
            return 0.0;

        if (m<0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14 
        }
    }
    else                    // modulo range: (y..0]
    {
        if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
            return 0;

        if (m>0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14 
        }
    }

    return m;
}

// wrap [rad] angle to [-PI..PI)
inline double WrapPosNegPI(double fAng)
{
    return modulus_fnc(fAng + _PI, _TWO_PI) - _PI;
}


namespace assisted_teleop {
  AssistedTeleop::AssistedTeleop() : costmap_ros_("local_costmap", tf_), planning_thread_(NULL){
    ros::NodeHandle private_nh("~");
    private_nh.param("controller_frequency", controller_frequency_, 10.0);
    private_nh.param("num_th_samples", num_th_samples_, 20);
    private_nh.param("num_x_samples", num_x_samples_, 10);
    private_nh.param("theta_range", theta_range_, 0.7);
    private_nh.param("translational_collision_speed", collision_trans_speed_, 0.0);
    private_nh.param("rotational_collision_speed", collision_rot_speed_, 0.0);
    private_nh.param("diff_drive", diff_drive_, true);
    
    planner_.initialize("TrajectoryPlannerROS", &tf_, &costmap_ros_);

    ros::NodeHandle n;
    pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    sub_ = n.subscribe("teleop_cmd_vel", 10, &AssistedTeleop::velCB, this);
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;

    planning_thread_ = new boost::thread(boost::bind(&AssistedTeleop::controlLoop, this));
  }

  AssistedTeleop::~AssistedTeleop(){
    planning_thread_->join();
    delete planning_thread_;
  }

  void AssistedTeleop::velCB(const geometry_msgs::TwistConstPtr& vel){
    boost::mutex::scoped_lock lock(mutex_);
    cmd_vel_ = *vel;
    
    if (true == diff_drive_)
    {
        cmd_vel_.linear.y = 0.0;
    }
  }

  void AssistedTeleop::controlLoop(){
    ros::Rate r(controller_frequency_);
    while(ros::ok()){
      Eigen::Vector3f desired_vel = Eigen::Vector3f::Zero();

      //we'll copy over odometry and velocity data for planning
      {
        boost::mutex::scoped_lock lock(mutex_);
        desired_vel[0] = cmd_vel_.linear.x;
        desired_vel[1] = cmd_vel_.linear.y;
        desired_vel[2] = cmd_vel_.angular.z;
      }

      //first, we'll check the trajectory that the user sent in... if its legal... we'll just follow it
      if(planner_.checkTrajectory(desired_vel[0], desired_vel[1], desired_vel[2], true)){
        geometry_msgs::Twist cmd;
        cmd.linear.x = desired_vel[0];
        cmd.linear.y = desired_vel[1];
        cmd.angular.z = desired_vel[2];
        pub_.publish(cmd);
        r.sleep();
        continue;
      }
      
      double dth = (theta_range_) / double(num_th_samples_);
      double dx = desired_vel[0] / double(num_x_samples_);
      double start_th = desired_vel[2] - theta_range_ / 2.0 ;

      Eigen::Vector3f best = Eigen::Vector3f::Zero();
      double best_dist = DBL_MAX;
      bool trajectory_found = false;
      Eigen::Vector3f diffs;
      double sq_dist;

      //if we don't have a valid trajectory... we'll start checking others in the angular range specified
      for(int i = 0; i < num_x_samples_; ++i){
        Eigen::Vector3f check_vel = Eigen::Vector3f::Zero();
        check_vel[0] = desired_vel[0] - i * dx;
        check_vel[1] = desired_vel[1];
        check_vel[2] = start_th;
        for(int j = 0; j < num_th_samples_; ++j){
          check_vel[2] = start_th + j * dth;
          check_vel[2] = WrapPosNegPI(check_vel[2]);
             
          if(planner_.checkTrajectory(check_vel[0], check_vel[1], check_vel[2], false)){
            //if we have a legal trajectory, we'll score it based on its distance to our desired velocity
            diffs = (desired_vel - check_vel);
            sq_dist = diffs[0] * diffs[0] + diffs[1] * diffs[1] + diffs[2] * diffs[2];

            //if we have a trajectory that is better than our best one so far, we'll take it
            if(sq_dist < best_dist){
              best = check_vel;
              best_dist = sq_dist;
              trajectory_found = true;
            }
          }
        }
      }
      
      if(!trajectory_found){
        //if we don't have a valid trajectory... we'll start checking others in the angular range specified
        for(int i = 0; i < num_x_samples_; ++i){
          Eigen::Vector3f check_vel = Eigen::Vector3f::Zero();
          check_vel[0] = desired_vel[0] - i * dx;
          check_vel[1] = desired_vel[1];
          check_vel[2] = start_th;
          for(int j = 0; j < num_th_samples_; ++j){
            check_vel[2] = start_th - j * dth;
            check_vel[2] = WrapPosNegPI(check_vel[2]);
               
            if(planner_.checkTrajectory(check_vel[0], check_vel[1], check_vel[2], false)){
              //if we have a legal trajectory, we'll score it based on its distance to our desired velocity
              diffs = (desired_vel - check_vel);
              sq_dist = diffs[0] * diffs[0] + diffs[1] * diffs[1] + diffs[2] * diffs[2];

              //if we have a trajectory that is better than our best one so far, we'll take it
              if(sq_dist < best_dist){
                best = check_vel;
                best_dist = sq_dist;
                trajectory_found = true;
              }
            }
          }
        }
      }

      //check if best is still zero, if it is... scale the original trajectory based on the collision_speed requested
      //but we only need to do this if the user has set a non-zero collision speed
      if(!trajectory_found && (collision_trans_speed_ > 0.0 || collision_rot_speed_ > 0.0)){
        double trans_scaling_factor = 0.0;
        double rot_scaling_factor = 0.0;
        double scaling_factor = 0.0;

        if(fabs(desired_vel[0]) > 0 && fabs(desired_vel[1]) > 0)
          trans_scaling_factor = std::min(collision_trans_speed_ / fabs(desired_vel[0]), collision_trans_speed_ / fabs(desired_vel[1]));
        else if(fabs(desired_vel[0]) > 0)
          trans_scaling_factor = collision_trans_speed_ / (fabs(desired_vel[0]));
        else if(fabs(desired_vel[1]) > 0)
          trans_scaling_factor = collision_trans_speed_ / (fabs(desired_vel[1]));

        if(fabs(desired_vel[2]) > 0)
          rot_scaling_factor = collision_rot_speed_ / (fabs(desired_vel[2]));

        if(collision_trans_speed_ > 0.0 && collision_rot_speed_ > 0.0)
          scaling_factor = std::min(trans_scaling_factor, rot_scaling_factor);
        else if(collision_trans_speed_ > 0.0)
          scaling_factor = trans_scaling_factor;
        else if(collision_rot_speed_ > 0.0)
          scaling_factor = rot_scaling_factor;

        //apply the scaling factor
        best = scaling_factor * best;
      }

      geometry_msgs::Twist best_cmd;
      best_cmd.linear.x = best[0];
      best_cmd.linear.y = best[1];
      best_cmd.angular.z = best[2];
      pub_.publish(best_cmd);

      r.sleep();
    }
  }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "movo_assisted_teleop");
  assisted_teleop::AssistedTeleop at;
  ros::spin();
  return 0;
}
