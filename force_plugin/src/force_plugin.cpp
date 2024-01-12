#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include "ros/ros.h"
//#include "gazebo/common/Color.hh"
#include <color.pb.h>
#include <angles/angles.h>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {

    private: ros::NodeHandle* _node_handle;
    private: ros::Publisher  _ft_pub;
    private: ros::Publisher  _ft_pub_x;
    private: ros::Publisher  _ft_pub_ang;
    private: ros::Publisher  _force_pub;
    private: ros::Subscriber  _actor_pos_sub;
    private: ros::Subscriber _human_hand_pose_sub;
    private: geometry_msgs::PointStamped _pose;
    private: float lh_x ;
		private: float lh_y ;
		private: float lh_z ;

    // Pointer to the update event connection
    
    private: physics::LinkPtr  _base_link;
    private: int _body_frame_ref;
    private: int _debug;
    private: ignition::transport::Node node_ign;
    private: ignition::msgs::Marker markerMsg;


    private: float force;
    private: float force_x; 

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      _node_handle = new ros::NodeHandle();	
      // Store the pointer to the model
      this->model = _parent;
  
      // Listen to the update event. This event is broadcast every
      // simulation iteration.

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      _ft_pub = _node_handle->advertise<std_msgs::Float64>("/pos_cmd", 1);
      _ft_pub_x = _node_handle->advertise<std_msgs::Float64>("/pos_cmd_x", 1);
      _ft_pub_ang = _node_handle->advertise<std_msgs::Float64>("/ang_cmd", 1);
      _force_pub = _node_handle->advertise<std_msgs::Float64>("/force_cmd", 1);
      _actor_pos_sub = _node_handle->subscribe("/actor_pos", 1, &ModelPush::pos_cb, this );
      _human_hand_pose_sub = _node_handle->subscribe("/human/hand_pose", 1, &ModelPush::leftHandCb, this);


      force=0;

      lh_x = 0.0;
      lh_y = 0.0;
      lh_z = 0.0;

           // Visual Marker

      double scale=0.1;

       markerMsg.set_ns("default");
       markerMsg.set_id(0);
       markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
       markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
       ignition::msgs::Set(markerMsg.mutable_scale(),
                       ignition::math::Vector3d(scale, scale, scale));

       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(1, 0.0, -0 ));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(1, 0.0, -0.1 ));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(15, 0.0, -0.1 ));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(1, 0.0, 0.1 ));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(15, 0.0, 0.1 ));

ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(1, 0.01, 0 ));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(1, 0.01, -0.1 ));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(15,  0.01, -0.1));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(1, 0.01, 0.1 ));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(15, 0.01, 0.1));
       
   
    
       
      
    }
    
    public: void pos_cb( geometry_msgs::PointStamped p ) {
      _pose = p;
    }

    public: void leftHandCb(geometry_msgs::Vector3 msg ) {

		lh_x = msg.x;
		lh_y = msg.y;
		lh_z = msg.z;
		
	  }
    // Called by the world update start event
    public: void OnUpdate()
    {  
      common::Time currTime = this->model->GetWorld()->SimTime();

       //Apply a small linear velocity to the model.

      _dog_base_link = model->GetLink("base_link");
      ignition::math::Pose3d base_pos=_dog_base_link->WorldPose(); 
      std_msgs::Float64 msg;
      std_msgs::Float64 msg_x;
      std_msgs::Float64 msg_force;
      std_msgs::Float64 msg_ang;

      float differencey=lh_y-base_pos.Pos().Y();
      float differencex=lh_x-base_pos.Pos().X();
      float differencez=lh_z-base_pos.Pos().Z();
      float difference=sqrt(pow(differencey,2)+pow(differencex,2)); 

      
      double alfa= angles::normalize_angle_positive(atan2(differencex,differencey));
      //double theta=atan2(differencez, sqrt(pow(differencey,2)+pow(differencex,2)));
      double tlc=30*(difference-0.6);
      
      //double tlc_x=30*(differencex-0.5*sin(alfa));
      
      //casE 1
      //if(currTime<60)
      //{
      //force=tlc;
      //force_x=tlc_x;
      //}
      //else if(currTime>=60 && currTime<70)
      //{
      //  force=53;
      //}

       //casE 2
      //if(currTime<30) 
      //{
      //force=tlc;
      //force_x=tlc_x;
      //}
      //else if(currTime>=30 && currTime<40)
      //{
      //  force=53;
      //}


        //casE 2
      //if(currTime<30)
      //{
      //force=tlc;
      //force_x=tlc_x;
      //}
      //else if(currTime>=30 && currTime<40)
      //{
      //  force=53;
      //}

      //case3
      //if(currTime<36 || currTime>36.5)
      //{
      force=tlc;
      //force_x=tlc_x;
      //}
      //else if(currTime>=36 && currTime<36.5)
      //{
      //  force=53;
      //}
     
     
     
      msg_force.data= force;
//
       if(force<0)
      {
        _dog_base_link->AddForceAtRelativePosition(ignition::math::Vector3d(0,0, 0),ignition::math::Vector3d(0,0,0));
      }
      else {
   // _dog_base_link->AddForceAtRelativePosition(ignition::math::Vector3d(force*sin(alfa),force*cos(alfa), 0),ignition::math::Vector3d(0,0,0));
    //  _dog_base_link->AddForceAtRelativePosition(ignition::math::Vector3d(0,force, 0),ignition::math::Vector3d(0,0,0));
      }



      if (force>=50) {
      
     // msg.data=0.0;
     // msg_x.data=0.0;
      ignition::msgs::Material *matMsg = markerMsg.mutable_material();     
      
      matMsg->mutable_script()->set_name("Gazebo/Red");
      
      }
      else if(force < 50 && force>=20)
      {

     // msg.data=30-force;
     // msg_x.data=(0.05+0.00065/force)*sin(alfa); //(1.7/force)*sin(alfa);
      
      ignition::msgs::Material *matMsg = markerMsg.mutable_material();
          matMsg->mutable_script()->set_name("Gazebo/Yellow");
      }
      else if( force <20 )
      {

     // msg.data=30-force;
     // msg_x.data=(0.05+0.5/force)*sin(alfa); //(1.7/force)*sin(alfa);

      ignition::msgs::Material *matMsg = markerMsg.mutable_material();
          matMsg->mutable_script()->set_name("Gazebo/Green");
      }
      
      

     ignition::math::Vector3d pos= _dog_base_link->WorldPose().Pos();	
    ignition::msgs::Set(markerMsg.mutable_pose(),
     ignition::math::Pose3d(pos.X(),pos.Y(),pos.Z()+0.02, 0, -1.57/7, 1.57-(alfa-0.16)));;

         //  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
         //  node_ign.Request("/marker", markerMsg);
    



    _ft_pub.publish( msg_force);

    _force_pub.publish( msg_force);
    
    }
    


    

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: physics::LinkPtr  _dog_base_link;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
