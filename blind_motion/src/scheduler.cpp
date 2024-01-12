#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
#include "eigen3/Eigen/Dense"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include "boost/thread.hpp"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "utils.hpp"

using namespace std;

class SCHEDULER {
    public:
        SCHEDULER();
        void run();
        void point_reached_cb( std_msgs::Bool msg );
        void main_loop();
        void modelStateCallback(const gazebo_msgs::ModelStates & msg);
        void publish_tf();

    private:
        ros::NodeHandle _nh;
        ros::Publisher  _wp_pub;
        ros::Publisher  _odom_pub;
        ros::Subscriber _point_reached_sub;
        ros::Subscriber _model_state_sub;
        bool _point_reached;
        bool _first_point_reached;
        std::vector< geometry_msgs::Vector3 > _wp_stack;
        Eigen::Vector2d _base_pos;
        Eigen::Vector4d _base_or;
        bool _first_wpose;
};


SCHEDULER::SCHEDULER() {

    _wp_pub = _nh.advertise< geometry_msgs::Vector3 > ("/apf/goal", 1);
    _point_reached_sub = _nh.subscribe("/blind_motion/point_reached", 1, &SCHEDULER::point_reached_cb, this);
    _model_state_sub = _nh.subscribe("/gazebo/model_states",1, &SCHEDULER::modelStateCallback,this);
    _odom_pub = _nh.advertise<nav_msgs::Odometry>("/odom", 1);

    _point_reached = true;
    _first_point_reached = false;

    geometry_msgs::Vector3 points;
    points.x = 10.0;
    points.y = 10.0;
    _wp_stack.push_back(points);

    points.x = 10.0;
    points.y = 10.0;
    _wp_stack.push_back(points);

    _first_wpose = false;

}


void SCHEDULER::point_reached_cb( std_msgs::Bool msg ) {

    //_point_reached = msg.data;
    _first_point_reached = true;

}


void SCHEDULER::publish_tf() {

    ros::Rate r(10);
    tf::TransformBroadcaster br;    
    tf::Transform transform;

    Matrix3f R; 
    Vector4f eig_q;
    Vector3f euler;


    nav_msgs::Odometry odom;

    while( ros::ok() ) {

        transform.setOrigin( tf::Vector3(-_base_pos[1], _base_pos[0], 0.0) );  

      
        eig_q << _base_or[0], _base_or[1], _base_or[2], _base_or[3];

        
        R = utilities::QuatToMat(eig_q);
        euler =  utilities::R2XYZ(R);


        /*
        euler[0] = 0.0;
        euler[1] = 0.0;
        euler[2] = euler[2];

        R = utilities::XYZ2R(euler);
        eig_q = utilities::rot2quat( R );
        */

        tf::Quaternion q(eig_q[1], eig_q[2], eig_q[3], eig_q[0]);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

        odom.pose.pose.orientation.w = eig_q[0];
        odom.pose.pose.orientation.x = eig_q[1];
        odom.pose.pose.orientation.y = eig_q[2];
        odom.pose.pose.orientation.z = eig_q[3];

        //_odom_pub.publish( odom );
        r.sleep();
    }
}
void SCHEDULER::modelStateCallback(const gazebo_msgs::ModelStates & msg) {
    
    string model_name = "laser";
    bool found = false;
    int index = 0;

    while(!found  && index < msg.name.size()){
        if( msg.name[index] == model_name ) found = true;
        else index++;
    }

    if(found){				
        _base_pos << msg.pose[index].position.x, msg.pose[index].position.y;
        _base_or  << msg.pose[index].orientation.w, msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z; 
        _first_wpose = true;


       
    }
}


void SCHEDULER::main_loop() {

    ros::Rate r(5);

    while (!_first_point_reached) usleep(0.1*1e6);

    //while( ros::ok() ) {
    bool done = false;
    int wp = 0;
    bool first = true;
    Eigen::Vector2d target;

    while( wp < _wp_stack.size() ) {

        if( first ) {
            cout << "Wp: " << wp << " sent!" << endl;
            _wp_pub.publish( _wp_stack[wp] );
            first = false;
            _point_reached = false;
        }

        target << _wp_stack[wp].x, _wp_stack[wp].y;
        if( (target - _base_pos).norm() < 0.1 ) _point_reached = true;

        if( _point_reached) {
            wp++;
            first = true;
        }
        r.sleep();
    }
    cout << "Done!" << endl;

}

void SCHEDULER::run() {

    boost::thread publish_tf_t( &SCHEDULER::publish_tf, this);
    //boost::thread main_loop_t(&SCHEDULER::main_loop, this);
    ros::spin();
}


int main( int argc, char** argv ) {

    ros::init( argc, argv, "blind_motion_scheduler");

    SCHEDULER sc;
    sc.run();

    return 0;
}