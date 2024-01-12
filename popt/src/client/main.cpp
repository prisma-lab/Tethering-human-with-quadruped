#include "iostream"
#include "../lopt.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <ros/package.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "boost/thread.hpp"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include <std_srvs/Empty.h>

#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include <map>
#include <unistd.h>
#include <unordered_map>
#include "../topt.h"
#include <angles/angles.h>
#include "geometry_msgs/WrenchStamped.h"
#include <geometry_msgs/PointStamped.h>

#include "gazebo_msgs/ContactsState.h"
#include <mutex>




std::mutex update_mutex;
std::mutex jstate_mutex;
std::mutex wpos_mutex;    


using namespace std;

enum SWING_LEGS { L1, L2, L3}; // 4 legs, br-fl, bl-fr
enum SWING_LEG { BR, FL, BL, FR}; // 4 legs, br-fl, bl-fr


class DOGCTRL {
    public:
        DOGCTRL();
        void jointStateCallback(const sensor_msgs::JointState & msg);
        void modelStateCallback(const gazebo_msgs::ModelStates & msg);
        void run();
        void update(Eigen::Matrix4d &eigenWorld_H_base, Eigen::Matrix<double,12,1> &eigenJointPos, Eigen::Matrix<double,12,1> &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity);			

        //Create the robot
        void createrobot(std::string modelFile);

        // Compute the Jacobian
        void  computeJac();
        void  ComputeJaclinear();

        // Compute matrix transformation T needed to recompute matrices/vecotor after the coordinate transform to the CoM
        void computeTransformation(const Eigen::VectorXd &Vel_);
        void computeJacDotQDot();
        void computeJdqdCOMlinear();
        void ctrl_loop();
        void publish_cmd(  Eigen::VectorXd tau  );
        void eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr);
        void eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl);
        void eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr);
        void eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl);
        void pos_cmd_cb(std_msgs::Float64 _pos_cmd);
        void pos_cmd_x_cb(std_msgs::Float64 _pos_cmd_x);
        void pos_cmd_ang_cb(std_msgs::Float64 _pos_cmd_ang);
        void estimate();
        void linear_vel_cb( std_msgs::Float64 msg);
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _joint_state_sub; 
        ros::Subscriber _model_state_sub; 
        ros::Subscriber _eebl_sub;
        ros::Subscriber _eebr_sub;
        ros::Subscriber _eefl_sub;
        ros::Subscriber _eefr_sub;
        ros::Subscriber _pos_cmd_sub;
        ros::Subscriber _pos_cmd_x_sub;
        ros::Subscriber _pos_cmd_ang_sub;
        ros::Subscriber _linear_vel_sub;

        ros::Publisher estimation_pub;
        ros::Publisher traj_des_pub;
        ros::Publisher traj_pub;
        ros::Publisher  _joint_pub;
             
        Eigen::Matrix<double,12,1> _jnt_pos; 
        Eigen::Matrix<double,12,1> _jnt_vel;
        Eigen::Matrix<double,6,1> _base_pos;
        Eigen::Matrix<double,6,1> _base_vel;
        Eigen::Matrix4d _world_H_base;

        double pos_cmd_x;
        double pos_cmd_y;
        double pos_cmd_ang;
        string _model_name;

        // Solve quadratic problem for contact forces
        Eigen::VectorXd qpproblem( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::Matrix<double,12,1> &fext);
        Eigen::VectorXd qpproblemtr( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::VectorXd vdotswdes, SWING_LEGS swinglegs, Eigen::Matrix<double,12,1> &fext);
        Eigen::VectorXd qpproblemol( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::Vector3d vdotswdes,  SWING_LEG swingleg, Eigen::Matrix<double,12,1> &fext);

        // get function
        Eigen::VectorXd getBiasMatrix();
        Eigen::VectorXd getGravityMatrix();
        Eigen::MatrixXd getMassMatrix();
        Eigen::MatrixXd getJacobian();
        Eigen::MatrixXd getBiasAcc();
        Eigen::MatrixXd getTransMatrix();
        Eigen::VectorXd getBiasMatrixCOM();
        Eigen::VectorXd getGravityMatrixCOM();
        Eigen::MatrixXd getMassMatrixCOM();
        Eigen::MatrixXd getMassMatrixCOM_com();
        Eigen::MatrixXd getMassMatrixCOM_joints();
        Eigen::MatrixXd getJacobianCOM();
        Eigen::MatrixXd getJacobianCOM_linear();
        Eigen::MatrixXd getBiasAccCOM();
        Eigen::MatrixXd getBiasAccCOM_linear();
        Eigen::MatrixXd getCOMpos();
        Eigen::MatrixXd getCOMvel();
        Eigen::MatrixXd getBRpos();
        Eigen::MatrixXd getBLpos();
        Eigen::MatrixXd getFLpos();
        Eigen::MatrixXd getFRpos();
        Eigen::MatrixXd getBRvel();
        Eigen::MatrixXd getBLvel();
        Eigen::MatrixXd getFLvel();
        Eigen::MatrixXd getFRvel();
        Eigen::MatrixXd getfest();
        Eigen::Matrix<double,3,3> getBRworldtransform();
        Eigen::Matrix<double,3,3> getBLworldtransform();
        Eigen::Matrix<double,3,3> getFLworldtransform();
        Eigen::Matrix<double,3,3> getFRworldtransform();
        Eigen::Matrix<double,3,1> getbrlowerleg();
        Eigen::MatrixXd getCtq();
        Eigen::MatrixXd getsolution();

        double getMass();
        int getDoFsnumber();

        // int for DoFs number
        unsigned int n;
        // Total mass of the robot
        double robot_mass;
        // KinDynComputations element
        iDynTree::KinDynComputations kinDynComp;
        // world to floating base transformation
        iDynTree::Transform world_H_base;
        // Joint position
        iDynTree::VectorDynSize jointPos;
        // Floating base velocity
        iDynTree::Twist         baseVel;
        // Joint velocity
        iDynTree::VectorDynSize jointVel;
        // Gravity acceleration
        iDynTree::Vector3       gravity; 
        // Position vector base+joints
        iDynTree::VectorDynSize  qb;
        // Velocity vector base+joints
        iDynTree::VectorDynSize  dqb;
        // Position vector COM+joints
        iDynTree::VectorDynSize  q;
        // Velocity vector COM+joints
        iDynTree::VectorDynSize  dq;
        // Joints limit vector
        iDynTree::VectorDynSize  qmin;
        iDynTree::VectorDynSize  qmax;
        // Center of Mass Position
        iDynTree::Vector6 CoM;
        // Center of mass velocity
        iDynTree::Vector6 CoM_vel;
        //Mass matrix
        iDynTree::FreeFloatingMassMatrix MassMatrix;
        //Bias Matrix
        iDynTree::VectorDynSize Bias;
        //Gravity Matrix
        iDynTree::MatrixDynSize GravMatrix;
        // Jacobian
        iDynTree::MatrixDynSize Jac;
        // Jacobian derivative
        iDynTree::MatrixDynSize JacDot;
        //CoM Jacobian
        iDynTree::MatrixDynSize Jcom;
        // Bias acceleration J_dot*q_dot
        iDynTree::MatrixDynSize Jdqd;
        // Transformation Matrix
        iDynTree::MatrixDynSize T;
        // Transformation matrix time derivative
        iDynTree::MatrixDynSize T_inv_dot;
        //Model
        iDynTree::Model model;
        iDynTree::ModelLoader mdlLoader;
        //Mass matrix in CoM representation
        iDynTree::FreeFloatingMassMatrix MassMatrixCOM;
        //Bias Matrix in CoM representation
        iDynTree::VectorDynSize BiasCOM;
        //Gravity Matrix in CoM representation
        iDynTree::MatrixDynSize GravMatrixCOM;
        // Jacobian in CoM representation
        iDynTree::MatrixDynSize JacCOM;
        //Jacobian in CoM representation (only linear part)
        iDynTree::MatrixDynSize JacCOM_lin;
        // Bias acceleration J_dot*q_dot in CoM representation
        iDynTree::MatrixDynSize JdqdCOM;
        // Bias acceleration J_dot*q_dot in CoM representation
        iDynTree::MatrixDynSize JdqdCOM_lin;

        Eigen::VectorXd x_eigen;
        Eigen::Matrix<double,3,1> fest; // party

        Eigen::Matrix<double,18,1> Ctq;
        void ComputeCtq(Eigen::Matrix<double,18,18> Ctq_pinocchio);
        Eigen::Matrix<double,18,18> Cm;
        bool _first_wpose;
        bool _first_jpos;
        unordered_map<int, string> _id2idname;  
        unordered_map<int, int> _id2index;      
        unordered_map<int, int> _index2id; 

        Eigen::Matrix<double,3,1>  force_br, force_bl, force_fl, force_fr;
        Eigen::Matrix<double,12,1> Fgrf;
        Eigen::VectorXd tau;
        std::vector<Eigen::Matrix<double,6,1>>  yd,  yw, w, ygamma, yd_prev,  yw_prev, w_prev, ygamma_prev;     

        OPT *_o;

        bool contact_br;
        bool contact_bl;
        bool contact_fl;
        bool contact_fr;
        bool flag_exit;
        double alfa;

        double gait_type;
        geometry_msgs::WrenchStamped estimation_msg;
        geometry_msgs::PointStamped p ; 
        float _linear_vel;
};



DOGCTRL::DOGCTRL() {

    _o = new OPT(30,86,82);
    _joint_state_sub = _nh.subscribe("/dogbot/joint_states", 1, &DOGCTRL::jointStateCallback, this);
    _model_state_sub = _nh.subscribe("/gazebo/model_states", 1, &DOGCTRL::modelStateCallback, this);
    _eebl_sub = _nh.subscribe("/dogbot/back_left_contactsensor_state",1, &DOGCTRL::eebl_cb, this);
    _eefl_sub = _nh.subscribe("/dogbot/front_left_contactsensor_state",1, &DOGCTRL::eefl_cb, this);
    _eebr_sub = _nh.subscribe("/dogbot/back_right_contactsensor_state",1, &DOGCTRL::eebr_cb, this);
    _eefr_sub = _nh.subscribe("/dogbot/front_right_contactsensor_state",1,&DOGCTRL::eefr_cb, this);
    _pos_cmd_sub = _nh.subscribe("/pos_cmd",1,&DOGCTRL::pos_cmd_cb, this);
    _pos_cmd_x_sub = _nh.subscribe("/pos_cmd_x",1,&DOGCTRL::pos_cmd_x_cb, this);
    _pos_cmd_ang_sub = _nh.subscribe("/ang_cmd",1,&DOGCTRL::pos_cmd_ang_cb, this);
    estimation_pub = _nh.advertise<geometry_msgs::WrenchStamped>("estimation_ee", 1);
    traj_des_pub = _nh.advertise<geometry_msgs::PointStamped>("traj_des", 1);
    traj_pub = _nh.advertise<geometry_msgs::PointStamped>("traj", 1);
    _linear_vel_sub = _nh.subscribe("/human/linear_vel", 1, &DOGCTRL::linear_vel_cb, this);

    _joint_pub = _nh.advertise<std_msgs::Float64MultiArray>("/dogbot/joint_position_controller/command", 1);
    _model_name = "dogbot";


    std::string path = ros::package::getPath("dogbot_description");
    path += "/urdf/dogbot.urdf";

    createrobot(path);

    model = kinDynComp.model();
	kinDynComp.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
	// Resize matrices of the class given the number of DOFs
    n = model.getNrOfDOFs();
    
    robot_mass = model.getTotalMass();
    jointPos = iDynTree::VectorDynSize(n);
    baseVel = iDynTree::Twist();
    jointVel = iDynTree::VectorDynSize(n);
	q = iDynTree::VectorDynSize(6+n);
	dq = iDynTree::VectorDynSize(6+n);
	qb = iDynTree::VectorDynSize(6+n);
	dqb=iDynTree::VectorDynSize(6+n);
	qmin= iDynTree::VectorDynSize(n);
	qmax= iDynTree::VectorDynSize(n);
	Bias=iDynTree::VectorDynSize(n+6);
	GravMatrix=iDynTree::MatrixDynSize(n+6,1);
    MassMatrix=iDynTree::FreeFloatingMassMatrix(model) ;
    Jcom=iDynTree::MatrixDynSize(3,6+n);
	Jac=iDynTree::MatrixDynSize(24,6+n);	
	JacDot=iDynTree::MatrixDynSize(24,6+n);
	Jdqd=iDynTree::MatrixDynSize(24,1);
    T=iDynTree::MatrixDynSize(6+n,6+n);
	T_inv_dot=iDynTree::MatrixDynSize(6+n,6+n);
    MassMatrixCOM=iDynTree::FreeFloatingMassMatrix(model) ;
    BiasCOM=iDynTree::VectorDynSize(n+6);
	GravMatrixCOM=iDynTree::MatrixDynSize(n+6,1);
	JacCOM=iDynTree::MatrixDynSize(24,6+n);
	JacCOM_lin=iDynTree::MatrixDynSize(12,6+n);
	JdqdCOM=iDynTree::MatrixDynSize(24,1);
	JdqdCOM_lin=iDynTree::MatrixDynSize(12,1);
	x_eigen= Eigen::VectorXd::Zero(30);
    
    tau.resize(12);
    tau= Eigen::VectorXd::Zero(12);
    //---

    yd.resize(4);  
    yw.resize(4); 
    w.resize(4); 
    ygamma.resize(4);
    yd_prev.resize(4);  
    yw_prev.resize(4); 
    w_prev.resize(4); 
    ygamma_prev.resize(4);

    std::fill(yd.begin(), yd.end(), Eigen::Matrix<double,6,1>::Zero()); 
    std::fill(yw.begin(), yw.end(), Eigen::Matrix<double,6,1>::Zero());
    std::fill(w.begin(), w.end(), Eigen::Matrix<double,6,1>::Zero());
    std::fill(ygamma.begin(), ygamma.end(), Eigen::Matrix<double,6,1>::Zero());
    std::fill(yd_prev.begin(), yd_prev.end(), Eigen::Matrix<double,6,1>::Zero()); 
    std::fill(yw_prev.begin(), yw_prev.end(), Eigen::Matrix<double,6,1>::Zero());
    std::fill(w_prev.begin(), w_prev.end(), Eigen::Matrix<double,6,1>::Zero());
    std::fill(ygamma_prev.begin(), ygamma_prev.end(), Eigen::Matrix<double,6,1>::Zero());

    _first_wpose = false;
    _first_jpos = false;
    contact_br = true; 
    contact_bl = true; 
    contact_bl = true; 
    contact_fr = true;
    flag_exit = false;
}


void DOGCTRL::linear_vel_cb( std_msgs::Float64 msg ) {
    _linear_vel = msg.data;
}

void DOGCTRL::createrobot(std::string modelFile) {  
    
    if( !mdlLoader.loadModelFromFile(modelFile) ) {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return ;
    }
    if( !kinDynComp.loadRobotModel(mdlLoader.model()) )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return ;
    }

    _id2idname.insert( pair< int, string > ( 0, kinDynComp.getDescriptionOfDegreeOfFreedom(0) ));
    _id2idname.insert( pair< int, string > ( 1, kinDynComp.getDescriptionOfDegreeOfFreedom(1) ));
    _id2idname.insert( pair< int, string > ( 2, kinDynComp.getDescriptionOfDegreeOfFreedom(2) ));
    _id2idname.insert( pair< int, string > ( 3, kinDynComp.getDescriptionOfDegreeOfFreedom(3) ));
    _id2idname.insert( pair< int, string > ( 4, kinDynComp.getDescriptionOfDegreeOfFreedom(4) ));
    _id2idname.insert( pair< int, string > ( 5, kinDynComp.getDescriptionOfDegreeOfFreedom(5) ));
    _id2idname.insert( pair< int, string > ( 6, kinDynComp.getDescriptionOfDegreeOfFreedom(6) ));
    _id2idname.insert( pair< int, string > ( 7, kinDynComp.getDescriptionOfDegreeOfFreedom(7) ));
    _id2idname.insert( pair< int, string > ( 8, kinDynComp.getDescriptionOfDegreeOfFreedom(8) ));
    _id2idname.insert( pair< int, string > ( 9, kinDynComp.getDescriptionOfDegreeOfFreedom(9) ));
    _id2idname.insert( pair< int, string > ( 10, kinDynComp.getDescriptionOfDegreeOfFreedom(10) ));
    _id2idname.insert( pair< int, string > ( 11, kinDynComp.getDescriptionOfDegreeOfFreedom(11) ));

}
// Get joints position and velocity
void DOGCTRL::jointStateCallback(const sensor_msgs::JointState & msg) {

    if( _first_jpos == false ) {
        for( int i=0; i<12; i++) {
            bool found = false;
            int index = 0;
            while( !found && index <  msg.name.size() ) {
                if( msg.name[index] == _id2idname.at( i )    ) {
                    found = true;

                    _id2index.insert( pair< int, int > ( i, index ));
                    _index2id.insert( pair< int, int > ( index, i ));

                }
                else index++;
            }
        }
    }

    for( int i=0; i<12; i++ ) {
        _jnt_pos( i, 0) = msg.position[    _id2index.at(i)    ];
    }

    for( int i=0; i<12; i++ ) {
        _jnt_vel( i, 0) = msg.velocity[    _id2index.at(i)    ];
    }
    
    _first_jpos = true;
}


// Get base position and velocity
void DOGCTRL::modelStateCallback(const gazebo_msgs::ModelStates & msg) {

    bool found = false;
    int index = 0;
    while( !found  && index < msg.name.size() ) {

        if( msg.name[index] == _model_name )
            found = true;
        else index++;
    }

    if( found ) {
        
        _world_H_base.setIdentity();
        
        //quaternion
        tf::Quaternion q(msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z,  msg.pose[index].orientation.w);
        q.normalize();
        Eigen::Matrix<double,3,3> rot;
        tf::matrixTFToEigen(tf::Matrix3x3(q),rot);

        //Roll, pitch, yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        //Set base pos (position and orientation)
        _base_pos << msg.pose[index].position.x, msg.pose[index].position.y, msg.pose[index].position.z, roll, pitch, yaw;
      
        //Set transformation matrix
        _world_H_base.block(0,0,3,3)= rot;
        _world_H_base.block(0,3,3,1)= _base_pos.block(0,0,3,1);

        //Set base vel
        _base_vel << msg.twist[index].linear.x, msg.twist[index].linear.y, msg.twist[index].linear.z, msg.twist[index].angular.x, msg.twist[index].angular.y, msg.twist[index].angular.z;
        _first_wpose = true;
    }
}


Eigen::Matrix<double,3,3> DOGCTRL::getBRworldtransform()
{    
	iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform(8);
    return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> DOGCTRL::getBLworldtransform()
{    
	iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform(11);
    return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> DOGCTRL::getFRworldtransform()
{    
	
	iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform(17);
    return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> DOGCTRL::getFLworldtransform()
{    
	
	iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform(14);
    return toEigen(World_br.getRotation());
}


// Compute matrix transformation T needed to recompute matrices/vector after the coordinate transform to the CoM
void DOGCTRL::computeTransformation(const Eigen::VectorXd &Vel_) {
    
    //Set ausiliary matrices
    iDynTree::MatrixDynSize Jb(6,6+n);
    iDynTree::MatrixDynSize Jbc(3,n);
    iDynTree::Vector3 xbc;
    iDynTree::MatrixDynSize xbc_hat(3,3);
    iDynTree::MatrixDynSize xbc_hat_dot(3,3);
    iDynTree::MatrixDynSize Jbc_dot(6,6+n);
    iDynTree::Vector3 xbo_dot;

    //Set ausiliary matrices
    iDynTree::Vector3 xbc_dot;

    // Compute T matrix
    // Get jacobians of the floating base and of the com
    kinDynComp.getFrameFreeFloatingJacobian(0,Jb);
    kinDynComp.getCenterOfMassJacobian(Jcom);

    // Compute jacobian Jbc=d(xc-xb)/dq used in matrix T
    toEigen(Jbc)<<toEigen(Jcom).block<3,12>(0,6)-toEigen(Jb).block<3,12>(0,6);

    // Get xb (floating base position) and xc ( com position)
    iDynTree::Position xb = world_H_base.getPosition();
    iDynTree::Position xc= kinDynComp.getCenterOfMassPosition();

    // Vector xcb=xc-xb
    toEigen(xbc)=toEigen(xc)-toEigen(xb);

    // Skew of xcb
    toEigen(xbc_hat)<<0, -toEigen(xbc)[2], toEigen(xbc)[1],
    toEigen(xbc)[2], 0, -toEigen(xbc)[0], -toEigen(xbc)[1], toEigen(xbc)[0], 0;

    Eigen::Matrix<double,6,6> X;
    X<<Eigen::MatrixXd::Identity(3,3), toEigen(xbc_hat).transpose(), 
    Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3);

    Eigen::MatrixXd Mb_Mj= toEigen(MassMatrix).block(0,0,6,6).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(toEigen(MassMatrix).block(0,6,6,12));
    Eigen::Matrix<double,6,12> Js=X*Mb_Mj;

    // Matrix T for the transformation
    toEigen(T)<<Eigen::MatrixXd::Identity(3,3), toEigen(xbc_hat).transpose(), Js.block(0,0,3,12),
    Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3), Js.block(3,0,3,12),
    Eigen::MatrixXd::Zero(12,3),  Eigen::MatrixXd::Zero(12,3), Eigen::MatrixXd::Identity(12,12);

    //Compute time derivative of T 
    // Compute derivative of xbc
    toEigen(xbc_dot)=toEigen(kinDynComp.getCenterOfMassVelocity())-toEigen(baseVel.getLinearVec3());
    Eigen::VectorXd  mdr=robot_mass*toEigen(xbc_dot);
    Eigen::Matrix<double,3,3> mdr_hat;
    mdr_hat<<0, -mdr[2], mdr[1],
    mdr[2], 0, -mdr[0],                          
    -mdr[1], mdr[0], 0;

    //Compute skew of xbc
    toEigen(xbc_hat_dot)<<0, -toEigen(xbc_dot)[2], toEigen(xbc_dot)[1],
    toEigen(xbc_dot)[2], 0, -toEigen(xbc_dot)[0],                          
    -toEigen(xbc_dot)[1], toEigen(xbc_dot)[0], 0;

    Eigen::Matrix<double,6,6> dX;
    dX<<Eigen::MatrixXd::Zero(3,3), toEigen(xbc_hat_dot).transpose(),
    Eigen::MatrixXd::Zero(3,6);
    // Time derivative of Jbc
    kinDynComp.getCentroidalAverageVelocityJacobian(Jbc_dot);

    Eigen::Matrix<double,6,6> dMb;
    dMb<<Eigen::MatrixXd::Zero(3,3), mdr_hat.transpose(),
    mdr_hat, Eigen::MatrixXd::Zero(3,3);

    Eigen::MatrixXd inv_dMb1=(toEigen(MassMatrix).block(0,0,6,6).transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dMb.transpose())).transpose();
    Eigen::MatrixXd inv_dMb2=-(toEigen(MassMatrix).block(0,0,6,6).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve( inv_dMb1));

    Eigen::Matrix<double,6,12> dJs=dX*Mb_Mj+X*inv_dMb2*toEigen(MassMatrix).block(0,6,6,12);

    toEigen(T_inv_dot)<<Eigen::MatrixXd::Zero(3,3), toEigen(xbc_hat_dot), -dJs.block(0,0,3,12),
    Eigen::MatrixXd::Zero(15,18);

}


//Update elements of the class given the new state
void DOGCTRL::update (Eigen::Matrix4d &eigenWorld_H_base, Eigen::Matrix<double,12,1> &eigenJointPos, Eigen::Matrix<double,12,1> &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity)
{   
   
    // Update joints, base and gravity from inputs
    iDynTree::fromEigen(world_H_base,eigenWorld_H_base);
    iDynTree::toEigen(jointPos) = eigenJointPos;
    iDynTree::fromEigen(baseVel,eigenBasevel);
    toEigen(jointVel) = eigenJointVel;
    toEigen(gravity)  = eigenGravity;

    Eigen::Vector3d worldeigen=toEigen(world_H_base.getPosition());

    while (worldeigen==Eigen::Vector3d::Zero()){
        iDynTree::fromEigen(world_H_base,eigenWorld_H_base);
        worldeigen=toEigen(world_H_base.getPosition());
        //cout<<"world update"<<world_H_base.getPosition().toString()<<endl;
    }

    //Set the state for the robot 
    kinDynComp.setRobotState(world_H_base,jointPos,
    baseVel,jointVel,gravity);

    // Compute Center of Mass
    iDynTree::Vector3 base_angle;
    toEigen(base_angle)=_base_pos.block(3,0,3,1);
    toEigen(CoM) << toEigen(kinDynComp.getCenterOfMassPosition()), toEigen(base_angle);
 

   
	//Compute velocity of the center of mass
	toEigen(CoM_vel)<<toEigen(kinDynComp.getCenterOfMassVelocity()), eigenBasevel.block(3,0,3,1);
		   
    // Compute position base +joints
	toEigen(qb)<<toEigen(world_H_base.getPosition()), toEigen(base_angle), eigenJointPos;
    // Compute position COM+joints
	toEigen(q)<<toEigen(CoM), eigenJointPos;
   	toEigen(dq)<<toEigen(CoM_vel), eigenJointVel;
	toEigen(dqb) << eigenBasevel, eigenJointVel;

	// Joint limits
    toEigen(qmin)<<-1.75 , -1.75,-1.75,-1.75,-1.58, -2.62, -3.15, -0.02,  -1.58, -2.62, -3.15, -0.02;
    toEigen(qmax)<<1.75, 1.75, 1.75, 1.75, 3.15, 0.02, 1.58, 2.62,  3.15, 0.02, 1.58, 2.62;

    // Get mass, bias (C(q,v)*v+g(q)) and gravity (g(q)) matrices
    //Initialize ausiliary vector
    iDynTree::FreeFloatingGeneralizedTorques bias_force(model);
    iDynTree::FreeFloatingGeneralizedTorques grav_force(model);
    //Compute Mass Matrix
    kinDynComp.getFreeFloatingMassMatrix(MassMatrix); 
    //Compute Coriolis + gravitational terms (Bias)
    kinDynComp.generalizedBiasForces(bias_force);
    toEigen(Bias)<<iDynTree::toEigen(bias_force.baseWrench()),
        iDynTree::toEigen(bias_force.jointTorques());


    //Compute Gravitational term
    kinDynComp.generalizedGravityForces(grav_force);
    toEigen(GravMatrix)<<iDynTree::toEigen(grav_force.baseWrench()),
                         iDynTree::toEigen(grav_force.jointTorques());

    computeJac();	
    // Compute Bias Acceleration -> J_dot*q_dot
    computeJacDotQDot();
    
    Eigen::Matrix<double, 18,1> q_dot;

    q_dot<< eigenBasevel,
    eigenJointVel;

    // Compute Matrix needed for transformation from floating base representation to CoM representation
    computeTransformation(q_dot);

    // Compute Mass Matrix in CoM representation 
    toEigen(MassMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T).inverse();

    // Compute Coriolis+gravitational term in CoM representation
    toEigen(BiasCOM)=toEigen(T).transpose().inverse()*toEigen(Bias)+toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T_inv_dot)*toEigen(dq);

    // Compute gravitational term in CoM representation	
    toEigen(GravMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(GravMatrix);

    // Compute Jacobian term in CoM representation
    toEigen(JacCOM)=toEigen(Jac)*toEigen(T).inverse();
    ComputeJaclinear();

    // Compute Bias Acceleration -> J_dot*q_dot  in CoM representation
    toEigen(JdqdCOM)=toEigen(Jdqd)+toEigen(Jac)*toEigen(T_inv_dot)*toEigen(dq);
    computeJdqdCOMlinear();
}


// Compute Jacobian
void  DOGCTRL::computeJac() {     

    //Set ausiliary matrices
    iDynTree::MatrixDynSize Jac1(6,6+n);
    iDynTree::MatrixDynSize Jac2(6,6+n);
    iDynTree::MatrixDynSize Jac3(6,6+n);
    iDynTree::MatrixDynSize Jac4(6,6+n);

    // Compute Jacobian for each leg
    // Jacobian for back right leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("back_right_foot"), Jac1);

    // Jacobian for back left leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("back_left_foot"),Jac2);

    // Jacobian for front left leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("front_left_foot"), Jac3);

    // Jacobian for front right leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("front_right_foot"), Jac4);

    // Full Jacobian
    toEigen(Jac)<<toEigen(Jac1), toEigen(Jac2), toEigen(Jac3), toEigen(Jac4);
    
}

void DOGCTRL::estimate() {

    Eigen::MatrixXd Mcom=toEigen(MassMatrixCOM).block(0,0,6,6);
    Eigen::Matrix<double,6,1> q_dot=toEigen(CoM_vel);
    Eigen::Matrix<double,12,6> J=toEigen(JacCOM_lin).block(0,0,12,6);

    Eigen::Matrix<double,6,1> fc=J.transpose()*Fgrf;
    Eigen::MatrixXd p1 = Mcom*q_dot;
    double mass_robot=model.getTotalMass();
    Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
    g_acc(2,0)=9.81;

    Eigen::MatrixXd intg=-mass_robot*g_acc+fc;
    Eigen::MatrixXd rho=p1;
    Eigen::MatrixXd d=intg;
    Eigen::VectorXd coeffs=Eigen::VectorXd::Zero(2);
    coeffs<< 10, 1;
    std::vector<Eigen::Matrix<double,6,6>> k(2);

    for (int i=0; i<1; i++) {
        k[i]=coeffs[i]*Eigen::Matrix<double,6,6>::Identity();
    }

    double T=0.005;
    Eigen::Matrix<double,6,6> m=(Eigen::Matrix<double,6,6>::Identity()+k[0]*T).inverse()*k[0];
    yd[0]=yd_prev[0]+(d*T);
    w[0] = m*(rho-yw_prev[0]-yd[0]);
    yw[0]=yw_prev[0]+w[0]*T;

    yd_prev =yd;
    yw_prev =yw;
    w_prev =w;
    ygamma_prev =ygamma;
}

void DOGCTRL::ComputeJaclinear() {
    
  Eigen::Matrix<double,12,24> B;
  B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

  toEigen(JacCOM_lin)=B*toEigen(JacCOM);
    
}

void DOGCTRL::computeJdqdCOMlinear() {
	
    Eigen::Matrix<double,12,24> B;
    B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);


    toEigen(JdqdCOM_lin)= Eigen::MatrixXd::Zero(12,1);
    toEigen(JdqdCOM_lin)=B*toEigen(JdqdCOM);
	
}

// Compute Bias acceleration: J_dot*q_dot
void  DOGCTRL::computeJacDotQDot() {

    // Bias acceleration for back right leg
    iDynTree::Vector6 Jdqd1=kinDynComp.getFrameBiasAcc("back_right_foot"); 
    // Bias acceleration for back left leg
    iDynTree::Vector6 Jdqd2=kinDynComp.getFrameBiasAcc("back_left_foot"); 
    // Bias acceleration for front left leg
    iDynTree::Vector6 Jdqd3=kinDynComp.getFrameBiasAcc("front_left_foot"); 
    // Bias acceleration for front right leg
    iDynTree::Vector6 Jdqd4=kinDynComp.getFrameBiasAcc("front_right_foot"); 
    toEigen(Jdqd)<<toEigen(Jdqd1), toEigen(Jdqd2), toEigen(Jdqd3), toEigen(Jdqd4);
	
}

void  DOGCTRL::publish_cmd(  Eigen::VectorXd tau  ) {
    std_msgs::Float64MultiArray tau1_msg;
    
    // Fill Command message
    for(int i=11; i>=0; i--) {
        tau1_msg.data.push_back(  tau( _index2id.at(i) )    );
    }

    //Sending command
    _joint_pub.publish(tau1_msg);

}


void DOGCTRL::pos_cmd_cb(std_msgs::Float64 _pos_cmd){
    pos_cmd_y = _pos_cmd.data;
}

void DOGCTRL::pos_cmd_x_cb(std_msgs::Float64 _pos_cmd){
    pos_cmd_x = _pos_cmd.data;
}

void DOGCTRL::pos_cmd_ang_cb(std_msgs::Float64 _pos_cmd_ang){
    pos_cmd_ang = _pos_cmd_ang.data;
}

void DOGCTRL::eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr){

	if(eebr->states.empty()){ 
        contact_br= false;
	}
	else{
		contact_br= true;
        force_br<<eebr->states[0].total_wrench.force.x, eebr->states[0].total_wrench.force.y, eebr->states[0].total_wrench.force.z;
	}
}

void DOGCTRL::eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl){
	if(eefl->states.empty()){ 
        contact_fl= false;
	}
	else{
		contact_fl= true;
        force_fl<<eefl->states[0].total_wrench.force.x, eefl->states[0].total_wrench.force.y, eefl->states[0].total_wrench.force.z;
    }
}

void DOGCTRL::eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl){

	if(eebl->states.empty()){ 
        contact_bl= false;
	}
	else{
		contact_bl= true;
        force_bl<<eebl->states[0].total_wrench.force.x, eebl->states[0].total_wrench.force.y, eebl->states[0].total_wrench.force.z;
    }
}

void DOGCTRL::eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr){
	if(eefr->states.empty()){ 
        contact_fr= false;
	}
	else {
		contact_fr= true;
        force_fr<<eefr->states[0].total_wrench.force.x, eefr->states[0].total_wrench.force.y, eefr->states[0].total_wrench.force.z;
	}
}
 
void DOGCTRL::ctrl_loop() {

    ros::ServiceClient pauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::ServiceClient unpauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty pauseSrv;
    _eebl_sub = _nh.subscribe("/dogbot/back_left_contactsensor_state",1, &DOGCTRL::eebl_cb, this);
    _eefl_sub = _nh.subscribe("/dogbot/front_left_contactsensor_state",1, &DOGCTRL::eefl_cb, this);
    _eebr_sub = _nh.subscribe("/dogbot/back_right_contactsensor_state",1, &DOGCTRL::eebr_cb, this);
    _eefr_sub = _nh.subscribe("/dogbot/front_right_contactsensor_state",1,&DOGCTRL::eefr_cb, this);

    //wait for first data...
    while( !_first_wpose  )
        usleep(0.1*1e6);

    while( !_first_jpos  )
        usleep(0.1*1e6);

    //update
    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.8;

    update_mutex.lock();
    update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);
    update_mutex.unlock();

    ros::Rate r(200);
    
    Eigen::VectorXd CoMPosDes;
    CoMPosDes = Eigen::VectorXd::Zero(6);

    //CoMAccDes
    std_msgs::Float64MultiArray tau1_msg;
    ros::Time begin;
    
    //feet
    iDynTree::Transform  World_bl;
	Eigen::MatrixXd eeBLposM;
    Eigen::Vector3d eeBLpos;
    Eigen::MatrixXd eeFLposM;
    Eigen::Vector3d eeFLpos;
    Eigen::MatrixXd eeFRposM;
    Eigen::Vector3d eeFRpos;
    Eigen::MatrixXd eeBRposM;
    Eigen::Vector3d eeBRpos;

    iDynTree::Vector6 des_com_pos;
    des_com_pos = CoM;
    toEigen(des_com_pos)[2] = 0.39;
    toEigen(des_com_pos)[3] = 0.0;
    toEigen(des_com_pos)[4] = 0.0;


  
    while( ros::ok() ) {

        towr::SplineHolder solution;
        towr::SplineHolder solution2;
        towr::NlpFormulation  formulation;
        towr::NlpFormulation  formulation2;

        
        des_com_pos = CoM;
        toEigen(des_com_pos)[2] = 0.4;
        toEigen(des_com_pos)[5] = 0.0;
        toEigen(des_com_pos)[0] += _linear_vel*0.5*sin(toEigen(des_com_pos)[5]);
        toEigen(des_com_pos)[1] += -_linear_vel*0.5*cos(toEigen(des_com_pos)[5]);


        CoMPosDes << toEigen(des_com_pos)[0],toEigen(des_com_pos)[1], toEigen(des_com_pos)[2], 
        toEigen(des_com_pos)[3], toEigen(des_com_pos)[4], toEigen(des_com_pos)[5];

        ros::Time now= ros::Time::now();
        p.header.stamp.sec=now.sec;
        p.header.stamp.nsec=now.nsec;              
        p.point.x = CoMPosDes[0];
        p.point.y = CoMPosDes[1];
        p.point.z = CoMPosDes[2];
        traj_des_pub.publish( p );

        p.point.x = toEigen(CoM)[0];
        p.point.y =  toEigen(CoM)[1];
        p.point.z =  toEigen(CoM)[2];
        traj_pub.publish( p );

        double pos_adm = (300*(30-sqrt(pow(w[0][0],2)+pow(w[0][1],2))));
        double deltat = 0.004;
        pos_adm = pos_adm*deltat*deltat;
        alfa = atan2(-w[0][0],-w[0][1]);

        CoMPosDes << toEigen(des_com_pos)[0], toEigen(des_com_pos)[1],  toEigen(des_com_pos)[2], 
        toEigen(des_com_pos)[3], toEigen(des_com_pos)[4], toEigen(des_com_pos)[5];

        World_bl = kinDynComp.getWorldTransform(   kinDynComp.getFrameIndex("back_left_foot")        );
	    Eigen::MatrixXd eeBLposM = toEigen(World_bl.getPosition());
        Eigen::Vector3d eeBLpos;
        eeBLpos << eeBLposM.block(0,0,3,1);

        World_bl = kinDynComp.getWorldTransform( kinDynComp.getFrameIndex("front_left_foot") );
        Eigen::MatrixXd eeFLposM = toEigen(World_bl.getPosition());
        Eigen::Vector3d eeFLpos;
        eeFLpos << eeFLposM.block(0,0,3,1);
        
        World_bl = kinDynComp.getWorldTransform( kinDynComp.getFrameIndex("front_right_foot") );
        Eigen::MatrixXd eeFRposM = toEigen(World_bl.getPosition());
        Eigen::Vector3d eeFRpos;
        eeFRpos << eeFRposM.block(0,0,3,1);

        World_bl = kinDynComp.getWorldTransform(  kinDynComp.getFrameIndex("back_right_foot") );
        Eigen::MatrixXd eeBRposM = toEigen(World_bl.getPosition());
        Eigen::Vector3d eeBRpos;
        eeBRpos << eeBRposM.block(0,0,3,1);
        
        Eigen::Matrix<double,6,6> Mcom;
        Mcom<< toEigen(MassMatrixCOM).block(0,0,6,6);
        if(pauseGazebo.call(pauseSrv))
            ROS_INFO("Simulation paused.");
        else
            ROS_INFO("Failed to pause simulation.");
    
        if(sqrt(pow(w[0][0],2)+pow(w[0][1],2))>50) {
            gait_type = 3;
                
            CoMPosDes << toEigen(des_com_pos)[0], toEigen(des_com_pos)[1],  toEigen(des_com_pos)[2], 
            toEigen(des_com_pos)[3], toEigen(des_com_pos)[4], toEigen(des_com_pos)[5];
        }
        else{
            gait_type = 1;
        }

        get_trajectory( toEigen( CoM ), toEigen(CoM_vel), CoMPosDes, eeBLpos, eeBRpos, eeFLpos, eeFRpos, gait_type , 0.5, solution, formulation, Mcom(3,3),Mcom(4,4) ,Mcom(5,5) ,Mcom(3,4), Mcom(3,5), Mcom(4,5) );
        unpauseGazebo.call(pauseSrv); 

        begin = ros::Time::now();

        while((ros::Time::now()-begin).toSec() <   formulation.params_.ee_phase_durations_.at(1)[0] ) {
            update_mutex.lock();
            update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);
            update_mutex.unlock();
        
            // Taking Jacobian for CoM and joints
            Eigen::Matrix<double, 12, 6> Jstcom= toEigen(JacCOM_lin).block(0,0,12,6);
            Eigen::Matrix<double, 12, 12> Jstj= toEigen(JacCOM_lin).block(0,6,12,12);
            Eigen::Matrix<double, 12, 18> Jst= toEigen(JacCOM_lin);
        
            // cost function quadratic matrix
            Eigen::Matrix<double,12,30> Sigma= Eigen::Matrix<double,12,30>::Zero();
            Sigma.block(0,18,12,12)= Eigen::Matrix<double,12,12>::Identity();
            Eigen::Matrix<double,6,30>  T_s= Jstcom.transpose()*Sigma;
            Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();
                
            Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
            Eigen::Matrix<double,30,30> eigenR= Eigen::Matrix<double,30,30>::Identity();
            Eigen::Matrix<double,30,30> eigenQ= eigenQ2+Eigen::Matrix<double,30,30>::Identity();
        
            // Compute deltax, deltav
            double t = (ros::Time::now()-begin).toSec();
            Eigen::Matrix<double,6,1> CoMPosD;
            CoMPosD << solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
            Eigen::Matrix<double,6,1> CoMVelD;
            CoMVelD << solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
            Eigen::Matrix<double,6,1> CoMAccD;
            CoMAccD << solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
            
            Eigen::Matrix<double,6,1> deltax = CoMPosD - toEigen( CoM );       
            deltax.block(3,0,3,1)<<_world_H_base.block(0,0,3,3)*deltax.block(3,0,3,1); 
            Eigen::Matrix<double,6,1> deltav = CoMVelD-toEigen(CoM_vel);
            Eigen::MatrixXd g_acc = Eigen::MatrixXd::Zero(6,1);
            g_acc(2,0) = 9.81;
                
            Eigen::MatrixXd M_com = toEigen(MassMatrixCOM).block(0,0,6,6);        
            Eigen::MatrixXd Kcom=3000*Eigen::MatrixXd::Identity(6,6);
            Eigen::MatrixXd Dcom=50*Eigen::MatrixXd::Identity(6,6);
                
            Eigen::Matrix<double,3,3> Tbr=getBRworldtransform();
            Eigen::Matrix<double,3,3> Tbl=getBLworldtransform();
            Eigen::Matrix<double,3,3> Tfl=getFLworldtransform();
            Eigen::Matrix<double,3,3> Tfr=getFRworldtransform();
            Fgrf<< Tbr*force_br, Tbl*force_bl,Tfl*force_fl,Tfr*force_fr;

            estimate();
            
            // Compute Desired vector
            Eigen::Matrix<double,6,1> Wcom_des = Kcom*deltax+Dcom*deltav+robot_mass*g_acc+toEigen(MassMatrixCOM).block(0,0,6,6)*CoMAccD-w[0];
            Eigen::Matrix<double,30,1> eigenc = -T_s.transpose()*eigenQ1.transpose()*Wcom_des;
        
            _o->setQ( eigenQ );
            _o->setc( eigenc );
        
            //Equality constraints
            Eigen::Matrix<double,18, 30> eigenA= Eigen::Matrix<double,18,30>::Zero();
            eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);
            eigenA.block(0,18,6,12)=-Jstcom.transpose();
            eigenA.block(6,0,12,6)=Jstcom;
            eigenA.block(6,6,12,12)=Jstj;

            // Known term
            Eigen::Matrix<double,18, 1> eigenb= Eigen::Matrix<double,18,1>::Zero();
            eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1);
            eigenb.block(6,0,12,1)=-toEigen(JdqdCOM_lin);
        
            //Inequality Constraints
            Eigen::Matrix<double,68, 30> eigenD= Eigen::Matrix<double,68,30>::Zero();
            
            // Torque limits
            eigenD.block(20,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);
            eigenD.block(20,18,12,12)=-Jstj.transpose();
            eigenD.block(32,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);
            eigenD.block(32,18,12,12)=Jstj.transpose();
            eigenD.block(44,6,12,12)=Eigen::Matrix<double,12,12>::Identity();
            eigenD.block(56,6,12,12)=-Eigen::Matrix<double,12,12>::Identity();
        
            //Friction
            double mu=0.6;
            Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
            n<< 0, 0, 1;

            Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
            t1<< 1, 0, 0;
        
            Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
            t2<<0, 1, 0;
        
            Eigen::Matrix<double,5,3> cfr=Eigen::Matrix<double,5,3>::Zero();
        
            cfr<<(-mu*n+t1).transpose(),
                    (-mu*n+t2).transpose(),
                    -(mu*n+t1).transpose(),
                    -(mu*n+t2).transpose(),
                    -n.transpose();
            
            Eigen::Matrix<double,20,12> Dfr=Eigen::Matrix<double,20,12>::Zero();
            for(int i=0; i<4; i++) {
                Dfr.block(0+5*i,0+3*i,5,3)=cfr;
            }
                
            eigenD.block(0,18,20,12)=Dfr;
            // Known terms for inequality
            Eigen::Matrix<double,68, 1> eigenC= Eigen::Matrix<double,68,1>::Zero();
        
            // Torque limits
            Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
            Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();
            Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);

            eigenC.block(20,0,12,1)=tau_max-eigenBiascom;
            eigenC.block(32,0,12,1)=-(tau_min-eigenBiascom);
        
            // Joints limits
            double deltat=0.025;
            Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
            Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
            Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
            Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
            Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
            Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

            eigenC.block(44,0,12,1)=ddqmax;
            eigenC.block(56,0,12,1)=-ddqmin;
        

            Eigen::Matrix<double,18,18> Si;
            Si<<Eigen::Matrix<double,6,18>::Zero(),
                Eigen::Matrix<double,12,6>::Zero(),Eigen::Matrix<double,12,12>::Identity();

        
            //Linear constraints matrix
            Eigen::Matrix<double,86, 31> eigenL= Eigen::Matrix<double,86,31>::Zero();

            eigenL<< eigenA,eigenb,
                        eigenD, eigenC;
            _o->setL_stance( eigenL );

            Eigen::VectorXd x_;
            x_.resize( 30 );
            _o->opt_stance( x_ );
                    
            tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_.block(6,0,12,1)+eigenBiascom-Jstj.transpose()*x_.block(18,0,12,1);
            publish_cmd( tau );

            estimation_msg.header.stamp = ros::Time::now();
            estimation_msg.wrench.force.x = w[0][0];
            estimation_msg.wrench.force.y =w[0][1];
            estimation_msg.wrench.force.z = w[0][2];
            estimation_msg.wrench.torque.x = w[0][3];
            estimation_msg.wrench.torque.y = w[0][4];
            estimation_msg.wrench.torque.z = w[0][5];

            estimation_pub.publish(estimation_msg);
        
            r.sleep();
        }

        flag_exit = false;
        
        if(gait_type ==1) {

            while((ros::Time::now()-begin).toSec() <  formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0] &  flag_exit == false) {
                
                update_mutex.lock();
                update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);
                update_mutex.unlock();
                double duration= formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0];
        
                // Taking Jacobian for CoM and joints
                int swl1, swl2, stl1, stl2;
                swl1=0;
                swl2=6 ;
                stl1=3;
                stl2=9 ;
                Eigen::Matrix<double, 6, 18> Jst= Eigen::Matrix<double,6,18>::Zero();
                Jst.block(0,0,3,18)=toEigen(JacCOM_lin).block(stl1,0,3,18);
                Jst.block(3,0,3,18)=toEigen(JacCOM_lin).block(stl2,0,3,18);
        
                Eigen::Matrix<double, 6, 18> Jsw= Eigen::Matrix<double,6,18>::Zero();
                Jsw.block(0,0,3,18)=toEigen(JacCOM_lin).block(swl1,0,3,18);
                Jsw.block(3,0,3,18)=toEigen(JacCOM_lin).block(swl2,0,3,18);
        
                // cost function quadratic matrix
                Eigen::Matrix<double,6,30> Sigma= Eigen::Matrix<double,6,30>::Zero();
                Sigma.block(0,18,6,6)= Eigen::Matrix<double,6,6>::Identity();
                
                Eigen::Matrix<double,6,30>  T_s= Jst.block(0,0,6,6).transpose()*Sigma;
                
                Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();
                
                Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
                Eigen::Matrix<double,30,30> eigenR= Eigen::Matrix<double,30,30>::Identity();
                
                eigenR.block(24,24,6,6)=100000000*Eigen::Matrix<double,6,6>::Identity();
                
                Eigen::Matrix<double,30,30> eigenQ= eigenQ2+eigenR;

                // Compute deltax, deltav
                double t = (ros::Time::now()-begin).toSec();
                Eigen::Matrix<double,6,1> CoMPosD;
                CoMPosD << solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
                Eigen::Matrix<double,6,1> CoMVelD;
                CoMVelD << solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
                Eigen::Matrix<double,6,1> CoMAccD;
                CoMAccD << solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
                
                Eigen::Matrix<double,6,1> deltax = CoMPosD - toEigen( CoM );        
                deltax.block(3,0,3,1)<<_world_H_base.block(0,0,3,3)*deltax.block(3,0,3,1);  
                //cout << "Errore di posizione: " << deltax.transpose() << endl;
                //cout << "ComPosD: " << CoMPosD << endl;
                //cout << "CoM: " << toEigen( CoM ) << endl;
                Eigen::Matrix<double,6,1> deltav = CoMVelD-toEigen(CoM_vel);
                Eigen::MatrixXd g_acc = Eigen::MatrixXd::Zero(6,1);
                g_acc(2,0)=9.81;
                
                Eigen::MatrixXd M_com = toEigen(MassMatrixCOM).block(0,0,6,6);        
                Eigen::MatrixXd Kcom=3000*Eigen::MatrixXd::Identity(6,6);
                Eigen::MatrixXd Dcom=50*Eigen::MatrixXd::Identity(6,6);


                Eigen::Matrix<double,3,3> Tbr= getBRworldtransform();
                Eigen::Matrix<double,3,3> Tbl= getBLworldtransform();
                Eigen::Matrix<double,3,3> Tfl= getFLworldtransform();
                Eigen::Matrix<double,3,3> Tfr= getFRworldtransform();
                Fgrf<< Eigen::Matrix<double,3,1>::Zero(), Tbl*force_bl, Eigen::Matrix<double,3,1>::Zero(), Tfr*force_fr;
                
                estimate();
                
                // Compute Desired vector
                Eigen::Matrix<double,6,1> Wcom_des = Kcom*deltax+Dcom*deltav+robot_mass*g_acc+toEigen(MassMatrixCOM).block(0,0,6,6)*CoMAccD-w[0];
                Eigen::Matrix<double,30,1> eigenc = -T_s.transpose()*eigenQ1.transpose()*Wcom_des;
        
                _o->setQ( eigenQ );
                _o->setc( eigenc );
        
        
                //Equality constraints
                Eigen::Matrix<double,12, 30> eigenA= Eigen::Matrix<double,12,30>::Zero();
                eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);
                eigenA.block(0,18,6,6)=-Jst.block(0,0,6,6).transpose();
                eigenA.block(6,0,6,6)=Jst.block(0,0,6,6);
                eigenA.block(6,6,6,12)=Jst.block(0,6,6,12);
        
                // Known term
                Eigen::Matrix<double,12, 1> eigenb= Eigen::Matrix<double,12,1>::Zero();
                Eigen::Matrix<double,6,1> Jdqdst= Eigen::Matrix<double,6,1>::Zero();
                Jdqdst<<toEigen(JdqdCOM_lin).block(stl1,0,3,1),
                        toEigen(JdqdCOM_lin).block(stl2,0,3,1);
            
                //Inequality Constraints
                Eigen::Matrix<double,70,30> eigenD= Eigen::Matrix<double,70,30>::Zero();
        
                    // Torque limits
                eigenD.block(10,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);   
                eigenD.block(10,18,12,6)=-Jst.block(0,6,6,12).transpose();
                eigenD.block(22,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);
                eigenD.block(22,18,12,6)=Jst.block(0,6,6,12).transpose();
                eigenD.block(34,0,3,6)=Jsw.block(0,0,3,6);        
                eigenD.block(34,6,3,12)=Jsw.block(0,6,3,12);
                eigenD.block(37,0,3,6)=Jsw.block(3,0,3,6);
                eigenD.block(37,6,3,12)=Jsw.block(3,6,3,12);
                eigenD.block(34,24,3,3)=-Eigen::Matrix<double,3,3>::Identity();
                eigenD.block(37,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();
                eigenD.block(40,0,3,6)=-Jsw.block(0,0,3,6);
                eigenD.block(40,6,3,12)=-Jsw.block(0,6,3,12);
                eigenD.block(43,0,3,6)=-Jsw.block(3,0,3,6);
                eigenD.block(43,6,3,12)=-Jsw.block(3,6,3,12);
                eigenD.block(40,24,3,3)=-Eigen::Matrix<double,3,3>::Identity();
                eigenD.block(43,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();
                eigenD.block(46,6,12,12)=Eigen::Matrix<double,12,12>::Identity();
                eigenD.block(58,6,12,12)=-Eigen::Matrix<double,12,12>::Identity();
                //Friction
                double mu=0.6;
                Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
                n<< 0,
                    0,
                    1;
        
                Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
                t1<< 1,
                    0,
                    0;
        
                Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
                t2<<0,
                    1,
                    0;
        
                Eigen::Matrix<double,5,3> cfr=Eigen::Matrix<double,5,3>::Zero();
        
                cfr<<(-mu*n+t1).transpose(),
                    (-mu*n+t2).transpose(),
                    -(mu*n+t1).transpose(),
                    -(mu*n+t2).transpose(),
                    -n.transpose();
                
                Eigen::Matrix<double,10,6> Dfr=Eigen::Matrix<double,10,6>::Zero();
        
                for(int i=0; i<2; i++)
                {
                    Dfr.block(0+5*i,0+3*i,5,3)=cfr;
                }
                    
            
                eigenD.block(0,18,10,6)=Dfr;
            
                
                // Known terms for inequality
                Eigen::Matrix<double,70, 1> eigenC= Eigen::Matrix<double,70,1>::Zero();
                
                // Torque limits
                Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
                Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();
            
                Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);
        
            
                eigenC.block(10,0,12,1)=tau_max-eigenBiascom;
                eigenC.block(22,0,12,1)=-(tau_min-eigenBiascom);
            
                // Joints limits
                double deltat=0.025;
                Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
                Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
                Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
                Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
                Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
                Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);
        
                eigenC.block(46,0,12,1)=ddqmax;
                eigenC.block(58,0,12,1)=-ddqmin;
        
        
                Eigen::Matrix<double,6,1> Jdqdsw= Eigen::Matrix<double,6,1>::Zero();
                Jdqdsw<<toEigen(JdqdCOM_lin).block(swl1,0,3,1),
                        toEigen(JdqdCOM_lin).block(swl2,0,3,1);    
                    
                Eigen::Matrix<double,6,1> accdes;

                accdes<< solution.ee_motion_.at(1)->GetPoint(t).a(),
                            solution.ee_motion_.at(2)->GetPoint(t).a();

                World_bl = kinDynComp.getWorldTransform("back_right_foot");
                Eigen::MatrixXd eeBRposM = toEigen(World_bl.getPosition());
                Eigen::Vector3d eeBRpos;
                eeBRpos << eeBRposM.block(0,0,3,1);

                World_bl = kinDynComp.getWorldTransform("front_left_foot");
                Eigen::MatrixXd eeFLposM = toEigen(World_bl.getPosition());
                Eigen::Vector3d eeFLpos;
                eeFLpos << eeFLposM.block(0,0,3,1);

                iDynTree::Twist br_vel;
                br_vel=kinDynComp.getFrameVel("back_right_foot");
                Eigen::MatrixXd eeBRvelM = toEigen(br_vel.getLinearVec3() );
                Eigen::Vector3d eeBRvel;
                eeBRvel << eeBRvelM.block(0,0,3,1);

                iDynTree::Twist fl_vel;
                fl_vel=kinDynComp.getFrameVel("front_left_foot");
                Eigen::MatrixXd eeFLvelM = toEigen(fl_vel.getLinearVec3() );
                Eigen::Vector3d eeFLvel;
                eeFLvel << eeFLvelM.block(0,0,3,1);


                Eigen::Matrix<double,6,1> posdelta;
                posdelta<< solution.ee_motion_.at(1)->GetPoint(t).p()-eeBRpos,
                        solution.ee_motion_.at(2)->GetPoint(t).p()-eeFLpos;

                //cout<<"posdes sw 1"<< solution.ee_motion_.at(1)->GetPoint(t).p()<<endl;
                //cout<<"posdes sw 2"<< solution.ee_motion_.at(2)->GetPoint(t).p()<<endl;
        
                Eigen::Matrix<double,6,1> veldelta;
                veldelta<< solution.ee_motion_.at(1)->GetPoint(t).v()-eeBRvel,
                        solution.ee_motion_.at(2)->GetPoint(t).v()-eeFLvel;
                
                Eigen::MatrixXd Kp;
                Kp=100*Eigen::MatrixXd::Identity(6,6);
                Eigen::MatrixXd Kd;
                Kd=5*Eigen::MatrixXd::Identity(6,6);
        
                Eigen::Matrix<double,6,1> vdotswdes=accdes+Kd*veldelta+Kp*posdelta;


                eigenC.block(34,0,6,1)= vdotswdes-Jdqdsw;
                eigenC.block(40,0,6,1)= -vdotswdes+Jdqdsw;
        
                
                //Linear constraints matrix
                Eigen::Matrix<double,82, 31> eigenL= Eigen::Matrix<double,82,31>::Zero();
        
                eigenL<< eigenA,eigenb,
                            eigenD, eigenC;
        
        
                _o->setL_swing( eigenL );
        
                Eigen::VectorXd x_;
                x_.resize( 30 );
                _o->opt_swing( x_ );
                
                
                tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_.block(6,0,12,1)+eigenBiascom-Jst.block(0,6,6,12).transpose()*x_.block(18,0,6,1);
                publish_cmd( tau );


                if( (contact_br==true || contact_fl==true) && t>duration-0.05)
                    {flag_exit=true;}
                
                estimation_msg.header.stamp = ros::Time::now();

                estimation_msg.wrench.force.x = w[0][0];
                estimation_msg.wrench.force.y =w[0][1];
                estimation_msg.wrench.force.z = w[0][2];
        
        
                estimation_msg.wrench.torque.x = w[0][3];
                estimation_msg.wrench.torque.y = w[0][4];
                estimation_msg.wrench.torque.z = w[0][5];
        
                estimation_pub.publish(estimation_msg);

                r.sleep();

            
            } 
        }

        CoMPosDes << toEigen(des_com_pos)[0], toEigen(des_com_pos)[1],  toEigen(des_com_pos)[2], 
            toEigen(des_com_pos)[3], toEigen(des_com_pos)[4], toEigen(des_com_pos)[5];
        
        now = ros::Time::now();

    
        p.header.stamp.sec=now.sec;
        p.header.stamp.nsec=now.nsec;  
        
        p.point.x = CoMPosDes[0];
        p.point.y =  CoMPosDes[1];
        p.point.z =  CoMPosDes[2];
    
        traj_des_pub.publish( p );
        p.point.x = toEigen(CoM)[0];
        p.point.y =  toEigen(CoM)[1];
        p.point.z =  toEigen(CoM)[2];

        traj_pub.publish( p );
        pos_adm=(300*(30-sqrt(pow(w[0][0],2)+pow(w[0][1],2))));
        pos_adm=pos_adm*deltat*deltat;
        alfa=atan2(-w[0][0],-w[0][1]);

        des_com_pos = CoM;
        
        toEigen(des_com_pos)[2] = 0.4;
        toEigen(des_com_pos)[5] = 0.0;
        toEigen(des_com_pos)[0] += _linear_vel*0.5*sin(toEigen(des_com_pos)[5]);
        toEigen(des_com_pos)[1] += -_linear_vel*0.5*cos(toEigen(des_com_pos)[5]);



        CoMPosDes << toEigen(des_com_pos)[0],toEigen(des_com_pos)[1],toEigen(des_com_pos)[2], 
        toEigen(des_com_pos)[3], toEigen(des_com_pos)[4], toEigen(des_com_pos)[5];




        World_bl = kinDynComp.getWorldTransform("back_left_foot");
        eeBLposM = toEigen(World_bl.getPosition());
        eeBLpos = eeBLposM.block(0,0,3,1);
        //cout << "Eeblpos " << eeBLpos << endl;

        World_bl = kinDynComp.getWorldTransform("front_left_foot");
        eeFLposM = toEigen(World_bl.getPosition());
        eeFLpos = eeFLposM.block(0,0,3,1);
        //cout << "Eeflpos " << eeFLpos << endl;

        World_bl = kinDynComp.getWorldTransform("front_right_foot");
        eeFRposM = toEigen(World_bl.getPosition());
        eeFRpos = eeFRposM.block(0,0,3,1);

        //cout << "Eefrpos " << eeFRpos << endl;

        World_bl = kinDynComp.getWorldTransform("back_right_foot");
        eeBRposM = toEigen(World_bl.getPosition());
        eeBRpos = eeBRposM.block(0,0,3,1);
        //cout << "Eebrpos " << eeBRpos << endl;

        Mcom<< toEigen(MassMatrixCOM).block(0,0,6,6);

        if(pauseGazebo.call(pauseSrv))
            ROS_INFO("Simulation paused.");
        else
            ROS_INFO("Failed to pause simulation.");

        
        //if(w[0][0]<50 && pos_cmd_y==0)
        if(sqrt(pow(w[0][0],2)+pow(w[0][1],2))>50)
        {
            gait_type=3;
            //CoMPosDes << toEigen(CoM)[0], toEigen(CoM)[1], 0.39, 0,  0, toEigen(CoM)[5];

            CoMPosDes << toEigen(des_com_pos)[0], toEigen(des_com_pos)[1],  0.39, 
            toEigen(des_com_pos)[3], toEigen(des_com_pos)[4], toEigen(des_com_pos)[5];

        }
        else{
            gait_type=2;
        }

        get_trajectory( toEigen( CoM ), toEigen(CoM_vel), CoMPosDes, eeBLpos, eeBRpos, eeFLpos, eeFRpos, gait_type, 0.5, solution2, formulation2, Mcom(3,3),Mcom(4,4) ,Mcom(5,5) ,Mcom(3,4), Mcom(3,5), Mcom(4,5) );

        unpauseGazebo.call(pauseSrv); 
        begin=ros::Time::now();
        while((ros::Time::now()-begin).toSec() <   formulation2.params_.ee_phase_durations_.at(0)[0] ) {

            update_mutex.lock();
            update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);
            update_mutex.unlock();

            // Taking Jacobian for CoM and joints
            Eigen::Matrix<double, 12, 6> Jstcom= toEigen(JacCOM_lin).block(0,0,12,6);
            Eigen::Matrix<double, 12, 12> Jstj= toEigen(JacCOM_lin).block(0,6,12,12);

            Eigen::Matrix<double, 12, 18> Jst= toEigen(JacCOM_lin);

            // cost function quadratic matrix
            Eigen::Matrix<double,12,30> Sigma= Eigen::Matrix<double,12,30>::Zero();
            Sigma.block(0,18,12,12)= Eigen::Matrix<double,12,12>::Identity();

            Eigen::Matrix<double,6,30>  T_s= Jstcom.transpose()*Sigma;
            Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();

            Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
            Eigen::Matrix<double,30,30> eigenR= Eigen::Matrix<double,30,30>::Identity();
            Eigen::Matrix<double,30,30> eigenQ= eigenQ2+Eigen::Matrix<double,30,30>::Identity();

            // Compute deltax, deltav
            double t = (ros::Time::now()-begin).toSec();
            Eigen::Matrix<double,6,1> CoMPosD;
            CoMPosD << solution2.base_linear_->GetPoint(t).p(), solution2.base_angular_->GetPoint(t).p();
            Eigen::Matrix<double,6,1> CoMVelD;
            CoMVelD << solution2.base_linear_->GetPoint(t).v(), solution2.base_angular_->GetPoint(t).v();
            Eigen::Matrix<double,6,1> CoMAccD;
            CoMAccD << solution2.base_linear_->GetPoint(t).a(), solution2.base_angular_->GetPoint(t).a();
            
            Eigen::Matrix<double,6,1> deltax = CoMPosD - toEigen( CoM );        
            deltax.block(3,0,3,1)<<_world_H_base.block(0,0,3,3)*deltax.block(3,0,3,1); 
            //cout << "Errore di posizione: " << deltax.transpose() << endl;
            //cout << "ComPosD: " << CoMPosD << endl;
            //cout << "CoM: " << toEigen( CoM ) << endl;
            Eigen::Matrix<double,6,1> deltav = CoMVelD-toEigen(CoM_vel);
            Eigen::MatrixXd g_acc = Eigen::MatrixXd::Zero(6,1);
            g_acc(2,0)=9.81;
            
            Eigen::MatrixXd M_com = toEigen(MassMatrixCOM).block(0,0,6,6);        
            Eigen::MatrixXd Kcom=3000*Eigen::MatrixXd::Identity(6,6);
            Eigen::MatrixXd Dcom=50*Eigen::MatrixXd::Identity(6,6);
            
            Eigen::Matrix<double,3,3> Tbr=getBRworldtransform();
            Eigen::Matrix<double,3,3> Tbl=getBLworldtransform();
            Eigen::Matrix<double,3,3> Tfl=getFLworldtransform();
            Eigen::Matrix<double,3,3> Tfr=getFRworldtransform();
            Fgrf<< Tbr*force_br, Tbl*force_bl,Tfl*force_fl,Tfr*force_fr;

            estimate();

            // Compute Desired vector
            Eigen::Matrix<double,6,1> Wcom_des = Kcom*deltax+Dcom*deltav+robot_mass*g_acc+toEigen(MassMatrixCOM).block(0,0,6,6)*CoMAccD-w[0];
            Eigen::Matrix<double,30,1> eigenc = -T_s.transpose()*eigenQ1.transpose()*Wcom_des;

            _o->setQ( eigenQ );
            _o->setc( eigenc );


            //Equality constraints
            Eigen::Matrix<double,18, 30> eigenA= Eigen::Matrix<double,18,30>::Zero();
            eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);
            eigenA.block(0,18,6,12)=-Jstcom.transpose();
            eigenA.block(6,0,12,6)=Jstcom;
            eigenA.block(6,6,12,12)=Jstj;

            // Known term
            Eigen::Matrix<double,18, 1> eigenb= Eigen::Matrix<double,18,1>::Zero();
            eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1);
            eigenb.block(6,0,12,1)=-toEigen(JdqdCOM_lin);
        
            //Inequality Constraints
            Eigen::Matrix<double,68, 30> eigenD= Eigen::Matrix<double,68,30>::Zero();
        
            // Torque limits
            eigenD.block(20,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);
            eigenD.block(20,18,12,12)=-Jstj.transpose();
            eigenD.block(32,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);
            eigenD.block(32,18,12,12)=Jstj.transpose();
            eigenD.block(44,6,12,12)=Eigen::Matrix<double,12,12>::Identity();
            eigenD.block(56,6,12,12)=-Eigen::Matrix<double,12,12>::Identity();
        
            //Friction
            double mu=0.6;
            Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
            n<< 0, 0, 1;

            Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
            t1<< 1, 0, 0;

            Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
            t2<<0, 1, 0;

            Eigen::Matrix<double,5,3> cfr=Eigen::Matrix<double,5,3>::Zero();
        
            cfr<<(-mu*n+t1).transpose(),
                    (-mu*n+t2).transpose(),
                    -(mu*n+t1).transpose(),
                    -(mu*n+t2).transpose(),
                    -n.transpose();
            
            Eigen::Matrix<double,20,12> Dfr=Eigen::Matrix<double,20,12>::Zero();

            for(int i=0; i<4; i++)
            {
                Dfr.block(0+5*i,0+3*i,5,3)=cfr;
            }
            

            eigenD.block(0,18,20,12)=Dfr;

            // Known terms for inequality
            Eigen::Matrix<double,68, 1> eigenC= Eigen::Matrix<double,68,1>::Zero();
        
            // Torque limits
            Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
            Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();
            Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);

            eigenC.block(20,0,12,1)=tau_max-eigenBiascom;
            eigenC.block(32,0,12,1)=-(tau_min-eigenBiascom);
        
            // Joints limits
            double deltat=0.025;
            Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
            Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
            Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
            Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
            Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
            Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

            eigenC.block(44,0,12,1)=ddqmax;
            eigenC.block(56,0,12,1)=-ddqmin;
        

            Eigen::Matrix<double,18,18> Si;
            Si<<Eigen::Matrix<double,6,18>::Zero(),
                Eigen::Matrix<double,12,6>::Zero(),Eigen::Matrix<double,12,12>::Identity();

        
            //Linear constraints matrix
            Eigen::Matrix<double,86, 31> eigenL= Eigen::Matrix<double,86,31>::Zero();

            eigenL<< eigenA,eigenb,
                        eigenD, eigenC;


            _o->setL_stance( eigenL );

            Eigen::VectorXd x_;
            x_.resize( 30 );
            _o->opt_stance( x_ );
            
            
            tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_.block(6,0,12,1)+eigenBiascom-Jstj.transpose()*x_.block(18,0,12,1);
            publish_cmd( tau );

            estimation_msg.header.stamp = ros::Time::now();

            estimation_msg.wrench.force.x = w[0][0];
            estimation_msg.wrench.force.y =w[0][1];
            estimation_msg.wrench.force.z = w[0][2];


            estimation_msg.wrench.torque.x = w[0][3];
            estimation_msg.wrench.torque.y = w[0][4];
            estimation_msg.wrench.torque.z = w[0][5];

            estimation_pub.publish(estimation_msg);

            r.sleep();
        }


        flag_exit=false;
        if(gait_type==2) {
            while((ros::Time::now()-begin).toSec() <  formulation2.params_.ee_phase_durations_.at(0)[0]+formulation2.params_.ee_phase_durations_.at(0)[1] &  flag_exit == false) {
            
                update_mutex.lock();
                update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);
                update_mutex.unlock();
                double duration=formulation2.params_.ee_phase_durations_.at(0)[0]+formulation2.params_.ee_phase_durations_.at(0)[1] ;
                // Taking Jacobian for CoM and joints
                int swl1, swl2, stl1, stl2;
                swl1=3;
                swl2=9 ;
                stl1=0;
                stl2=6 ;
                Eigen::Matrix<double, 6, 18> Jst= Eigen::Matrix<double,6,18>::Zero();
                Jst.block(0,0,3,18)=toEigen(JacCOM_lin).block(stl1,0,3,18);
                Jst.block(3,0,3,18)=toEigen(JacCOM_lin).block(stl2,0,3,18);

                Eigen::Matrix<double, 6, 18> Jsw= Eigen::Matrix<double,6,18>::Zero();
                Jsw.block(0,0,3,18)=toEigen(JacCOM_lin).block(swl1,0,3,18);
                Jsw.block(3,0,3,18)=toEigen(JacCOM_lin).block(swl2,0,3,18);

                // cost function quadratic matrix
                Eigen::Matrix<double,6,30> Sigma= Eigen::Matrix<double,6,30>::Zero();
                Sigma.block(0,18,6,6)= Eigen::Matrix<double,6,6>::Identity();
            
                Eigen::Matrix<double,6,30>  T_s= Jst.block(0,0,6,6).transpose()*Sigma;
        
                Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();
                Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
                Eigen::Matrix<double,30,30> eigenR= Eigen::Matrix<double,30,30>::Identity();
                
                eigenR.block(24,24,6,6)=100000000*Eigen::Matrix<double,6,6>::Identity();
                
                Eigen::Matrix<double,30,30> eigenQ= eigenQ2+eigenR;

                // Compute deltax, deltav
                double t = (ros::Time::now()-begin).toSec();
                Eigen::Matrix<double,6,1> CoMPosD;
                CoMPosD << solution2.base_linear_->GetPoint(t).p(), solution2.base_angular_->GetPoint(t).p();
                Eigen::Matrix<double,6,1> CoMVelD;
                CoMVelD << solution2.base_linear_->GetPoint(t).v(), solution2.base_angular_->GetPoint(t).v();
                Eigen::Matrix<double,6,1> CoMAccD;
                CoMAccD << solution2.base_linear_->GetPoint(t).a(), solution2.base_angular_->GetPoint(t).a();
                
                Eigen::Matrix<double,6,1> deltax = CoMPosD - toEigen( CoM );      
                deltax.block(3,0,3,1)<<_world_H_base.block(0,0,3,3)*deltax.block(3,0,3,1);  
                //cout << "Errore di posizione: " << deltax.transpose() << endl;
                //cout << "ComPosD: " << CoMPosD << endl;
                //cout << "CoM: " << toEigen( CoM ) << endl;

                Eigen::Matrix<double,6,1> deltav = CoMVelD-toEigen(CoM_vel);
                Eigen::MatrixXd g_acc = Eigen::MatrixXd::Zero(6,1);
                g_acc(2,0)=9.81;
                
                Eigen::MatrixXd M_com = toEigen(MassMatrixCOM).block(0,0,6,6);        
                Eigen::MatrixXd Kcom=3000*Eigen::MatrixXd::Identity(6,6);
                Eigen::MatrixXd Dcom=50*Eigen::MatrixXd::Identity(6,6);
                
                Eigen::Matrix<double,3,3> Tbr=getBRworldtransform();
                Eigen::Matrix<double,3,3> Tbl=getBLworldtransform();
                Eigen::Matrix<double,3,3> Tfl=getFLworldtransform();
                Eigen::Matrix<double,3,3> Tfr=getFRworldtransform();

                Fgrf<<Tbr*force_br, Eigen::Matrix<double,3,1>::Zero(),  Tfl*force_fl, Eigen::Matrix<double,3,1>::Zero();
                
                estimate();
                
                // Compute Desired vector
                Eigen::Matrix<double,6,1> Wcom_des = Kcom*deltax+Dcom*deltav+robot_mass*g_acc+toEigen(MassMatrixCOM).block(0,0,6,6)*CoMAccD-w[0];
                Eigen::Matrix<double,30,1> eigenc = -T_s.transpose()*eigenQ1.transpose()*Wcom_des;
        
                _o->setQ( eigenQ );
                _o->setc( eigenc );
        
        
                //Equality constraints
                Eigen::Matrix<double,12, 30> eigenA= Eigen::Matrix<double,12,30>::Zero();
                eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);
                eigenA.block(0,18,6,6)=-Jst.block(0,0,6,6).transpose();
                eigenA.block(6,0,6,6)=Jst.block(0,0,6,6);
                eigenA.block(6,6,6,12)=Jst.block(0,6,6,12);
        
                // Known term
                Eigen::Matrix<double,12, 1> eigenb= Eigen::Matrix<double,12,1>::Zero();
                Eigen::Matrix<double,6,1> Jdqdst= Eigen::Matrix<double,6,1>::Zero();
                Jdqdst<<toEigen(JdqdCOM_lin).block(stl1,0,3,1),
                        toEigen(JdqdCOM_lin).block(stl2,0,3,1);
            
                //Inequality Constraints
                Eigen::Matrix<double,70,30> eigenD= Eigen::Matrix<double,70,30>::Zero();
        
                // Torque limits
                eigenD.block(10,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);
                eigenD.block(10,18,12,6)=-Jst.block(0,6,6,12).transpose();
                eigenD.block(22,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);
                eigenD.block(22,18,12,6)=Jst.block(0,6,6,12).transpose();
                eigenD.block(34,0,3,6)=Jsw.block(0,0,3,6);
                eigenD.block(34,6,3,12)=Jsw.block(0,6,3,12);
                eigenD.block(37,0,3,6)=Jsw.block(3,0,3,6);
                eigenD.block(37,6,3,12)=Jsw.block(3,6,3,12);
                eigenD.block(34,24,3,3)=-Eigen::Matrix<double,3,3>::Identity();
                eigenD.block(37,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();
                eigenD.block(40,0,3,6)=-Jsw.block(0,0,3,6);
                eigenD.block(40,6,3,12)=-Jsw.block(0,6,3,12);
                eigenD.block(43,0,3,6)=-Jsw.block(3,0,3,6);
                eigenD.block(43,6,3,12)=-Jsw.block(3,6,3,12);
                eigenD.block(40,24,3,3)=-Eigen::Matrix<double,3,3>::Identity();
                eigenD.block(43,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();
                eigenD.block(46,6,12,12)=Eigen::Matrix<double,12,12>::Identity();
                eigenD.block(58,6,12,12)=-Eigen::Matrix<double,12,12>::Identity();
            
                //Friction
                double mu=0.6;
                Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
                n<< 0, 
                    0,
                    1;
        
                Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
                t1<< 1,
                    0,
                    0;
        
                Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
                t2<<0,
                    1,
                    0;
        
                Eigen::Matrix<double,5,3> cfr=Eigen::Matrix<double,5,3>::Zero();
        
                cfr << (-mu*n+t1).transpose(),
                            (-mu*n+t2).transpose(),
                            -(mu*n+t1).transpose(),
                            -(mu*n+t2).transpose(),
                            -n.transpose();
                        
                Eigen::Matrix<double,10,6> Dfr=Eigen::Matrix<double,10,6>::Zero();
                for(int i=0; i<2; i++) {
                    Dfr.block(0+5*i,0+3*i,5,3)=cfr;
                }
                eigenD.block(0,18,10,6)=Dfr;
                // Known terms for inequality
                Eigen::Matrix<double,70, 1> eigenC= Eigen::Matrix<double,70,1>::Zero();
            
                // Torque limits
                Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
                Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones(); 
                Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);
                eigenC.block(10,0,12,1)=tau_max-eigenBiascom;
                eigenC.block(22,0,12,1)=-(tau_min-eigenBiascom);
            
                // Joints limits
                double deltat=0.025;
                Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
                Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
                Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
                Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
                Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
                Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);
        
                eigenC.block(46,0,12,1)=ddqmax;
                eigenC.block(58,0,12,1)=-ddqmin;
                
                
                Eigen::Matrix<double,6,1> Jdqdsw= Eigen::Matrix<double,6,1>::Zero();
                Jdqdsw<<toEigen(JdqdCOM_lin).block(swl1,0,3,1),toEigen(JdqdCOM_lin).block(swl2,0,3,1);    
                Eigen::Matrix<double,6,1> accdes;
        
                accdes<< solution2.ee_motion_.at(0)->GetPoint(t).a(), solution2.ee_motion_.at(3)->GetPoint(t).a();
                World_bl = kinDynComp.getWorldTransform("back_left_foot");
                Eigen::MatrixXd eeBLposM = toEigen(World_bl.getPosition());
                Eigen::Vector3d eeBLpos;
                eeBLpos << eeBLposM.block(0,0,3,1);
        
                World_bl = kinDynComp.getWorldTransform("front_right_foot");
                Eigen::MatrixXd eeFRposM = toEigen(World_bl.getPosition());
                Eigen::Vector3d eeFRpos;
                eeFRpos << eeFRposM.block(0,0,3,1);

                iDynTree::Twist br_vel;
                br_vel=kinDynComp.getFrameVel("back_left_foot");
                Eigen::MatrixXd eeBLvelM = toEigen(br_vel.getLinearVec3() );
                Eigen::Vector3d eeBLvel;
                eeBLvel << eeBLvelM.block(0,0,3,1);

                iDynTree::Twist fl_vel;
                fl_vel=kinDynComp.getFrameVel("front_right_foot");
                Eigen::MatrixXd eeFRvelM = toEigen(fl_vel.getLinearVec3() );
                Eigen::Vector3d eeFRvel;
                eeFRvel << eeFRvelM.block(0,0,3,1);

        
                Eigen::Matrix<double,6,1> posdelta;
                posdelta<< solution2.ee_motion_.at(0)->GetPoint(t).p()-eeBLpos,
                        solution2.ee_motion_.at(3)->GetPoint(t).p()-eeFRpos;
        
                Eigen::Matrix<double,6,1> veldelta;
                veldelta<< solution2.ee_motion_.at(0)->GetPoint(t).v()-eeBLvel,
                        solution2.ee_motion_.at(3)->GetPoint(t).v()-eeFRvel;
                
                Eigen::MatrixXd Kp;
                Kp=100*Eigen::MatrixXd::Identity(6,6);
                Eigen::MatrixXd Kd;
                Kd=5*Eigen::MatrixXd::Identity(6,6);       
                Eigen::Matrix<double,6,1> vdotswdes=accdes+Kd*veldelta+Kp*posdelta;
        
                eigenC.block(34,0,6,1)= vdotswdes-Jdqdsw;
                eigenC.block(40,0,6,1)= -vdotswdes+Jdqdsw;
            
                //Linear constraints matrix
                Eigen::Matrix<double,82, 31> eigenL= Eigen::Matrix<double,82,31>::Zero();
            
                eigenL<< eigenA,eigenb,
                            eigenD, eigenC;
            
            
                _o->setL_swing( eigenL );
            
                Eigen::VectorXd x_;
                x_.resize( 30 );
                _o->opt_swing( x_ );
                    
                Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);
                tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_.block(6,0,12,1)+eigenBiascom-Jst.block(0,6,6,12).transpose()*x_.block(18,0,6,1);
                publish_cmd( tau );

                estimation_msg.header.stamp = ros::Time::now();
                estimation_msg.wrench.force.x = w[0][0];
                estimation_msg.wrench.force.y =w[0][1];
                estimation_msg.wrench.force.z = w[0][2];
                        
                estimation_msg.wrench.torque.x = w[0][3];
                estimation_msg.wrench.torque.y = w[0][4];
                estimation_msg.wrench.torque.z = w[0][5];
                
                estimation_pub.publish(estimation_msg);

            
                r.sleep();
            
                if( (contact_fr==true || contact_bl==true) && t>duration-0.05) {
                    flag_exit=true;
                }       
            } 
        }
    }
}


void DOGCTRL::run() {
    ros::ServiceClient set_model_configuration_srv = _nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient set_model_state_srv = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    
    gazebo_msgs::SetModelConfiguration robot_init_config;
    robot_init_config.request.model_name = "dogbot";
    robot_init_config.request.urdf_param_name = "robot_description";
    robot_init_config.request.joint_names.push_back("back_left_roll_joint");
    robot_init_config.request.joint_names.push_back("back_left_pitch_joint");
    robot_init_config.request.joint_names.push_back("back_left_knee_joint");
    robot_init_config.request.joint_names.push_back("back_right_roll_joint");
    robot_init_config.request.joint_names.push_back("back_right_pitch_joint");
    robot_init_config.request.joint_names.push_back("back_right_knee_joint");
    robot_init_config.request.joint_names.push_back("front_left_roll_joint");
    robot_init_config.request.joint_names.push_back("front_left_pitch_joint");
    robot_init_config.request.joint_names.push_back("front_left_knee_joint");
    robot_init_config.request.joint_names.push_back("front_right_roll_joint");
    robot_init_config.request.joint_names.push_back("front_right_pitch_joint");
    robot_init_config.request.joint_names.push_back("front_right_knee_joint");
    robot_init_config.request.joint_positions.push_back( 0.0004875394147498824);
    robot_init_config.request.joint_positions.push_back( -0.884249947977489);
    robot_init_config.request.joint_positions.push_back(-1.6039026405138666);
    robot_init_config.request.joint_positions.push_back( 0.0006243098169198547);
    robot_init_config.request.joint_positions.push_back(0.8861978063639038);
    robot_init_config.request.joint_positions.push_back(1.6032646991719783);
    robot_init_config.request.joint_positions.push_back(-3.197670677312914e-05);
    robot_init_config.request.joint_positions.push_back(-0.8848124990461947);
    robot_init_config.request.joint_positions.push_back(-1.6039627256817717);
    robot_init_config.request.joint_positions.push_back(-0.0005127385581351618);
    robot_init_config.request.joint_positions.push_back(0.886353788084274);
    robot_init_config.request.joint_positions.push_back( 1.60361055049274);

    if(set_model_configuration_srv.call(robot_init_config))
        ROS_INFO("Robot configuration set.");
    else
        ROS_INFO("Failed to set robot configuration.");


    gazebo_msgs::SetModelState robot_init_state;
    robot_init_state.request.model_state.model_name = "dogbot";
    robot_init_state.request.model_state.reference_frame = "world";
    robot_init_state.request.model_state.pose.position.x = 1.00;
    robot_init_state.request.model_state.pose.position.y = -1.0;
    robot_init_state.request.model_state.pose.position.z = 0.430159040502;
    robot_init_state.request.model_state.pose.orientation.x=0.0;
    robot_init_state.request.model_state.pose.orientation.y=0.0;
    robot_init_state.request.model_state.pose.orientation.z=0;
    robot_init_state.request.model_state.pose.orientation.w=1;
    if(set_model_state_srv.call(robot_init_state))
        ROS_INFO("Robot state set.");
    else
        ROS_INFO("Failed to set robot state.");



    boost::thread ctrl_loop_t( &DOGCTRL::ctrl_loop, this );
    ros::spin();
}

int main(int argc, char** argv) {

    ros::init( argc, argv, "popt");
    DOGCTRL dc;
    dc.run();

}
