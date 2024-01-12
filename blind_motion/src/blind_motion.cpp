#include "ros/ros.h"
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
#include "std_msgs/Bool.h"

using namespace std;

class APF{

    public:
        APF();
        void lidar_cb(const sensor_msgs::LaserScan & msg);
        void modelStateCallback(const gazebo_msgs::ModelStates & msg);

        Eigen::Vector2d compute_att_force(Eigen::Vector2d & err);
        Eigen::Vector2d compute_rep_force();
        bool check_local_minima(Eigen::Vector2d &err);
        void apf_goal();
        
        void run();
        void goal_cb( geometry_msgs::Vector3 msg );
    private:
        
        ros::NodeHandle _nh;
        
        ros::Subscriber _lidar_sub;
        ros::Subscriber _goal_sub;
        vector<float> _obstacle_x;
	    vector<float> _obstacle_y;
        double _ray=0;
        ros::Subscriber _model_state_sub;
        Eigen::Matrix<double,6,1> _base_pos;
        Eigen::Matrix<double,6,1> _base_vel;
        bool _first_wpose;
        
        Eigen::Vector2d startpoint;
        Eigen::Vector2d _goal_point;

        double _old_yaw;
        int _turns;

        double _Ka; 
        double _Kr;

        double min_threshold,err_threshold;
        float _eta_0;
        float _min_ob_dist;
        double att_pot,rep_pot,tot_pot=0;
        Eigen::Vector2d att_force,rep_force,tot_force;

        ros::Publisher _cmd_pub;
        ros::Publisher _wp_reached_pub;
        string _robot_name;
        string _lidar_topic;
        string _cmd_vel_topic;
        bool _new_goal;      
};

APF::APF(){




    //---Load params
    if( !_nh.getParam("cmd_vel_topic", _cmd_vel_topic) ) {
        _cmd_vel_topic =  "/cmd_vel";
    }
    if( !_nh.getParam("robot_name", _robot_name) ) {
        _robot_name =  "burger";
    }
    if( !_nh.getParam("lidar_topic", _lidar_topic) ) {
        _lidar_topic =  "/lidar";
    }
    

    cout << "cmd_vel_topic: " << _cmd_vel_topic << endl;
    cout << "robot_name: " << _robot_name << endl;
    cout << "lidar_topic: " << _lidar_topic << endl;



    _lidar_sub = _nh.subscribe(_lidar_topic.c_str(),1, &APF::lidar_cb, this);
    _model_state_sub = _nh.subscribe("/gazebo/model_states",1, &APF::modelStateCallback,this);
    _goal_sub = _nh.subscribe("/apf/goal", 1, &APF::goal_cb, this);
    _wp_reached_pub = _nh.advertise<std_msgs::Bool> ("/blind_motion/point_reached", 1);
    _cmd_pub = _nh.advertise<geometry_msgs::Twist>(_cmd_vel_topic.c_str(), 1);
    
    
    _first_wpose = false;
    _Ka = 40;
    _Kr = 20;
    _old_yaw = 0;
    _turns=0;
    _eta_0=1.9;
    _min_ob_dist = 1.5;
    _new_goal = false;
    /*
    yaw_room,
    min_threshold=28;
    err_threshold=1;
    startpoint=Eigen::VectorXd::Zero(2);
    att_force,rep_force,tot_force=Eigen::VectorXd::Zero(2);
    */
}



void APF::goal_cb( geometry_msgs::Vector3 msg ) {
    _goal_point << msg.x, msg.y;
    _new_goal = true;
}

Eigen::Vector2d APF::compute_rep_force(){


    Eigen::Vector2d gradient = Eigen::VectorXd::Zero(2);
    //return gradient;

    float min_eta = 50;

    for(int i=0; i<_obstacle_x.size(); i++){
        
        float eta_i = sqrt(pow(_obstacle_x[i],2)+pow(_obstacle_y[i],2));
        
        if(eta_i < min_eta){
            min_eta = eta_i;
            gradient << -_obstacle_x[i]/eta_i, -_obstacle_y[i]/eta_i;
        }
    }

    _eta_0 = 1.9; //??
    if(min_eta <= _eta_0){
        rep_pot = _Kr/2*pow((1/min_eta -1/_eta_0),2);
        rep_force=(_Kr/pow(min_eta,2))*(1/min_eta-1/_eta_0)*gradient;
    }
    else{
        rep_pot=0;
        rep_force=Eigen::VectorXd::Zero(2);
    }
    
    cout << "Min eta: " << min_eta << endl;
    cout << "rep_force: " << rep_force.transpose() << endl; 
    return rep_force;
}

Eigen::Vector2d APF::compute_att_force(Eigen::Vector2d & err){
	if(err.norm()>1){
        att_pot = _Ka*err.norm();
        att_force = _Ka*(err/err.norm()); 
    }
	else {
        att_pot= 0.5*_Ka * pow(err.norm(),2);
        att_force = _Ka*err;
    }
    return att_force;
}

void APF::lidar_cb(const sensor_msgs::LaserScan & msg){


    if( !_first_wpose ) return;

    sensor_msgs::LaserScan laser = msg;
    double alpha = 0;
    _ray = 0;
    
    _obstacle_x.clear();
	_obstacle_y.clear();

    for(int i=0;i<laser.ranges.size();i++){
        

        //int ob = i; //laser.ranges.size() / 2;
        int ob = laser.ranges.size() / 2;
        //cout << "ob: " << ob << " " << laser.ranges[ob] << endl;
        if (  laser.ranges[ob] < _min_ob_dist ) {

        
            alpha = ob*laser.angle_increment;

            if ( alpha > 0.8 && alpha < 1.8 ) {
                _ray = laser.ranges[ob];     
                double x,y;
                x = _ray*cos(alpha + _base_pos(5) -M_PI/2.0); 
                y = _ray*sin(alpha + _base_pos(5) -M_PI/2.0);
                
                
                _obstacle_x.push_back(x);
                _obstacle_y.push_back(y);
                
                cout << "Ray: " << _ray << endl;
                cout << "alpha: " << alpha << endl;
                cout << "Obs in: " << x << ", " << y << endl;            
            }
        }
    }
}


void APF::modelStateCallback(const gazebo_msgs::ModelStates & msg) {
    
    string model_name = _robot_name;
    bool found = false;
    int index = 0;

    while(!found  && index < msg.name.size()){
        if( msg.name[index] == model_name ) found = true;
        else index++;
    }

    if(found){						
        //quaternion
        tf::Quaternion Q(msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z,  msg.pose[index].orientation.w);
        Q.normalize();

        
        //Roll, pitch, yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(Q).getRPY(roll, pitch, yaw);



        if( yaw * _old_yaw<0 && abs(yaw)>3){
            if(yaw < _old_yaw) _turns ++;
            else _turns--;
        }
        _old_yaw = yaw;
        yaw += _turns*6.28;


        yaw -= 1.57;

        //Set base pos (position and orientation)
        _base_pos << msg.pose[index].position.x, msg.pose[index].position.y, msg.pose[index].position.z, roll, pitch, yaw;
        //Set base vel
        

        //cout << "_base_pos: " << _base_pos.transpose() << endl;
        _base_vel << msg.twist[index].linear.x, msg.twist[index].linear.y, msg.twist[index].linear.z, msg.twist[index].angular.x, msg.twist[index].angular.y, msg.twist[index].angular.z;
        _first_wpose = true;
    }
    
}



void APF::apf_goal() { 

    ros::Rate rate(100);

	double desired_yaw;
    double _old_yaw;
    Eigen::Vector2d current_position, current_velocity;
    double current_yaw;
    //Eigen::Vector2d pos_error;
	double ang_error;
	double dt = 1.0/100.0;
    Eigen::Vector2d pos_goal_min1, pos_goal_min2;

	Eigen::Vector2d error=Eigen::VectorXd::Zero(2);
	att_force,rep_force,tot_force=Eigen::VectorXd::Zero(2);
    att_pot,rep_pot,tot_pot=0;

    
    bool reachedgoal = false; 
    bool min_loop=false;
    bool random=false;

    _base_pos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	while(!_first_wpose) usleep(0.1*1e6); 
    
    current_position(0) = _base_pos(0);
    current_position(1) = _base_pos(1);
	current_yaw = _base_pos(5);
    desired_yaw = current_yaw;
	
    double linear_vel = 0.0;
    double angular_vel = 0.0;
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;

    _cmd_pub.publish(cmd_vel);
	
    
    Eigen::Vector2d goal_point;
    goal_point << current_position(0), current_position(1);
    

    std_msgs::Bool wp_reached_data;
    wp_reached_data.data = false;
    while (ros::ok() ) {

        if ( _new_goal ) {
            goal_point = _goal_point;
            _new_goal = false;
            wp_reached_data.data = false;
        }
        
		current_position(0) = _base_pos(0);
        current_position(1) = _base_pos(1);
        current_yaw = _base_pos(5);

        // LOCAL MINIMA
        /*
        if(check_local_minima(error) && !min_loop){
            min_loop=true;
            if(abs(rep_force(0))>abs(rep_force(1))){
                pos_goal_min1(0)=(rep_force(0)>0)? current_position(0)+1.5 : current_position(0)-1.5;
                pos_goal_min1(1)=current_position(1)-0.5;
                pos_goal_min2(0)=(rep_force(0)>0)? pos_goal_min1(0)-(rand()%3+1.5): pos_goal_min1(0)+rand()%3+1.5;
                //pos_goal_min2(1)= (rep_force(1)>0)? pos_goal_min1(1) + rand()%2 : pos_goal_min1(1)-rand()%2;
                pos_goal_min2(1)= pos_goal_min1(1) - 1;
            }
            else{
                pos_goal_min1(0)=current_position(0)-0.5;
                pos_goal_min1(1)=(rep_force(1)>0)? current_position(1)+1.5: current_position(1)-1.5;
                pos_goal_min2(0)= pos_goal_min1(0) - 1;
                //pos_goal_min2(0)= (rep_force(0)>0)? pos_goal_min1(0)+rand()%2 : pos_goal_min1(0)-rand()%2;
                pos_goal_min2(1)= (rep_force(1)>0)? pos_goal_min1(1)-(rand() %3+1.5): pos_goal_min1(1)+rand() %3+1.5;
            }   
            ROS_INFO("POS GOAL MIN 1: %f,%f",pos_goal_min1[0],pos_goal_min1[1]);
            ROS_INFO("POS GOAL MIN 2: %f,%f",pos_goal_min2[0],pos_goal_min2[1]);
        }
        */
        
        /*
        if(min_loop){
            //ROS_INFO("LOCAL MIN");
            if(!random){
                error = pos_goal_min1 - current_position;
                if(error.norm()<0.4){
                    ROS_INFO_ONCE("POS GOAL MIN 2: %f,%f",pos_goal_min2[0],pos_goal_min2[1]);
                    random=true;
                }
                att_force= compute_att_force(error);
                rep_force= compute_rep_force();
                tot_force= att_force + rep_force;
                desired_yaw= current_yaw;
            }
            else{
                error = pos_goal_min2 - current_position;
                if(error.norm()<0.5){
                    random=false;
                    min_loop=false;
                } 
                att_force=compute_att_force(error);
                rep_force= compute_rep_force();
                tot_force = att_force + rep_force;
                desired_yaw=atan2(tot_force(1), tot_force(0)) + 1.57;
                if(old_yaw*desired_yaw<0 && current_yaw>3) desired_yaw=desired_yaw+6.28;
                old_yaw=desired_yaw;
            }
        }
        else{
        */

        //pos_goal = current_position;
        //pos_goal(0) += 0.0;
        //pos_goal(1) += 1.0;
        
        error = goal_point - current_position;
        if(error.norm()<0.15) {
            reachedgoal = true;
            //wp_reached_data.data = true; 
        }

        att_force = compute_att_force(error);
        rep_force = compute_rep_force();

        if( att_force.norm() == 0 ) {
            rep_force << 0.0, 0.0;
        }


        tot_force = att_force + rep_force;
        

        //tot_force = 
        desired_yaw = atan2(tot_force(1), tot_force(0));
        
        
        ang_error = desired_yaw - current_yaw;

        cout << "Pos error: " << error.transpose() << endl;
        cout << "att_force: " << att_force.transpose() << endl;
        cout << "rep_force: " << rep_force.transpose() << endl;
        cout << "tot_force: " << tot_force.transpose() << endl;
        cout << "desired_yaw: " << desired_yaw << " / " << current_yaw << endl;

        
        if( fabs(ang_error) >= 0.18) {
            angular_vel = 0.5*ang_error;
            linear_vel = 0.0;
        } 
        else {
            
            angular_vel = 0.5*ang_error;
            //linear_vel = tot_force[0]*dt;
            linear_vel = tot_force.norm()*dt;
        }

        
        /*
		if (pos_error.squaredNorm()>0.01){
            // cout<<"Pos_Error!"<<endl; 
            cmd_pos.x=desired_position(0);
            cmd_pos.y=desired_position(1);
		}
		else{
           // cout<<"Apf increment"<<endl;
            desired_velocity= current_velocity + tot_force*dt;
            desired_position= current_position + desired_velocity*dt;
            cmd_pos.x=desired_position(0); 
            cmd_pos.y=desired_position(1);
		}
        */
        
        // Turn around if ang_error > ca 10deg
		if(fabs(ang_error)>=0.18){  
            // cout<<"Turn around"<<endl;
            //desired_position = current_position;
            //cmd_pos.x=desired_position(0);
            //cmd_pos.y=desired_position(1);
            //cmd_pos.yaw = current_yaw + 0.1*ang_error/abs(ang_error);
        }
        else {
            //cout << "qui!" << endl;
            //cmd_pos.yaw=current_yaw;
        }
    
        /*cmd_pos.x=current_position(0);
        cmd_pos.y=current_position(1);
		desired_position = current_position;
        cmd_pos.yaw = current_yaw;*/
    /*
        cmd_pub.publish(cmd_pos);
	*/

        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;

        _cmd_pub.publish(cmd_vel);
        _wp_reached_pub.publish( wp_reached_data );
    	rate.sleep();
	
    
    }
    
    cmd_vel.linear.x =  0.0;
    cmd_vel.angular.z = 0.0;

    _cmd_pub.publish(cmd_vel);
	
}



void APF::run() {

    Eigen::Vector2d goal;
    goal << 1.0, 1.0;

    boost::thread apf_goal_t( &APF::apf_goal, this );
    ros::spin();
}


int main( int argc, char** argv ) {

    ros::init( argc, argv, "blind_motion");

    APF apf;
    apf.run();
    return 0;
}