#include <string>
#include <ros/ros.h>
#include <vector>
#include <tf/tf.h>
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/LaserScan.h"
#include "boost/thread.hpp"
#include "gazebo_msgs/ModelState.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Twist.h"
#include "tf_conversions/tf_eigen.h"
#include "utils.hpp"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace Eigen;

class SET_LPOSE {
    public:
        SET_LPOSE();
        void run();
        void modelStateCallback(const gazebo_msgs::ModelStates & msg);
        void cmd_vel_cb( geometry_msgs::Twist msg ) ;
        void lidar_cb(const sensor_msgs::LaserScan & msg);
        void navigate();
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _model_state_sub;
        ros::Subscriber _cmd_vel_sub;
        ros::Publisher _model_state_pub;
        ros::Publisher _human_vel_pub;
        ros::Publisher _human_pose_pub;
        Eigen::Matrix<double,3,3> _laser_w_rot;
        double _l_yaw;
        geometry_msgs::Twist _laser_vel_data;
        ros::Publisher _hvel_pub;


};

SET_LPOSE::SET_LPOSE() {
    _model_state_sub  = _nh.subscribe("/gazebo/model_states", 1, &SET_LPOSE::modelStateCallback, this );
    _cmd_vel_sub      = _nh.subscribe("/cmd_vel", 1, &SET_LPOSE::cmd_vel_cb, this);
    _human_vel_pub    = _nh.advertise<geometry_msgs::Twist>("/human/cmd_vel", 1);
    _model_state_pub  = _nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    _human_pose_pub   = _nh.advertise<geometry_msgs::Pose>("/human/cmd_pos", 1);
    _hvel_pub         = _nh.advertise<std_msgs::Float64>("/human/linear_vel", 1);
    
}

void SET_LPOSE::cmd_vel_cb( geometry_msgs::Twist msg ) {
    _laser_vel_data = msg;
}

void SET_LPOSE::modelStateCallback(const gazebo_msgs::ModelStates & msg) {

    bool found = false;
    int index = 0;
    std::string _human_model_name="laser";

    while( !found  && index < msg.name.size() ) {

        if( msg.name[index] == _human_model_name )
            found = true;
        else index++;
    }

    if( found ) {    
        tf::Quaternion q(msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z,  msg.pose[index].orientation.w);
        q.normalize();
        
        tf::matrixTFToEigen(tf::Matrix3x3(q), _laser_w_rot);
        double l_roll, l_pitch;
        tf::Matrix3x3(q).getRPY(l_roll, l_pitch, _l_yaw);
        //_l_yaw = -_l_yaw;
    }
}


void SET_LPOSE::navigate() {

    ros::Rate r(50);

    gazebo_msgs::ModelState mstate;
    Eigen::Vector3d i_pos;
    i_pos << 0.0, 0.0, 0.0;
    double dt = 1.0/50.0;
    Eigen::Matrix<double,3,3> _l_rot;

    Eigen::Vector3d b_vel; 
    mstate.pose.position.x = i_pos[0];

    geometry_msgs::Twist h_vel;
    geometry_msgs::Pose  h_pos;
    std_msgs::Float64    h_vel_todog;

    while( ros::ok() ) {

        mstate.model_name = "laser";
        
        _l_rot <<   cos( _l_yaw ), -sin(_l_yaw ),   0,
                    sin( _l_yaw ),  cos( _l_yaw ),  0,
                    0,              0,              1;

        b_vel << _laser_vel_data.linear.x, 0.0, 0.0;
        b_vel =  _l_rot.transpose()*b_vel;
  
        //transform.setOrigin( tf::Vector3(-_base_pos[1], _base_pos[0], 0.0) );  


        mstate.pose.position.x += -b_vel[1]*dt;
        mstate.pose.position.y += -b_vel[0]*dt;
        mstate.pose.position.z = 0.6;
        
        Vector3f rpy; 
        rpy[2] = _l_yaw + (_laser_vel_data.angular.z)*dt;
        //rpy[2] = -rpy[2];

        Matrix3f R = utilities::XYZ2R( rpy );
        Vector4f quat = utilities::rot2quat( R ); 
        
        mstate.pose.orientation.x = quat[1];
        mstate.pose.orientation.y = quat[2];
        mstate.pose.orientation.z = quat[3];
        mstate.pose.orientation.w = quat[0];
        
        h_pos = mstate.pose; 
        
        h_vel_todog.data = b_vel[0];
        
        _hvel_pub.publish( h_vel_todog );
        _model_state_pub.publish( mstate );        
        _human_pose_pub.publish( h_pos );
        r.sleep();
    }

}

void SET_LPOSE::run() {
    boost::thread navigate_t(&SET_LPOSE::navigate, this);   
    ros::spin();
}


int main( int argc, char** argv ) {

    ros::init(argc, argv, "set_laser_pose");
    SET_LPOSE lp;
    lp.run();


    return 0;
}