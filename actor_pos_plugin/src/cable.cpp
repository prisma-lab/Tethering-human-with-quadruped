#include <gazebo/common/Plugin.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include <thread>
#include <string> 

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
// if you want some positions of the model use this....
#include <gazebo_msgs/ModelStates.h>
#include "iostream"
#include "cmath"


using namespace std;


namespace gazebo
{
  namespace rendering
  {
    class visual_plugin : public VisualPlugin
    {
      public:
  
        visual_plugin();
        virtual ~visual_plugin();
        void Load( VisualPtr _parent, sdf::ElementPtr _sdf );
		void leftHandCb(geometry_msgs::Vector3 msg);

      protected: 

        virtual void UpdateChild();
        virtual void Init();

      private:

        VisualPtr visual_;
        ScenePtr scene_;
        DynamicLines *line;
        std::string visual_namespace_;
        event::ConnectionPtr update_connection_;
        
        double x_offset = 0.012171;
        double y_offset = 0.001913;
        double z_offset = 0.109754;
        //double r_offset = -0.0065;
        double r_offset = 0;
        double x;
        double y;
        double z;
        ignition::math::Vector3d  poles;
        bool init = false;
      	char nr = '1';     
		float xline = 0.0;
		physics:: ModelPtr actor_model;
        ros::NodeHandle* _node_handle;
		ros::Subscriber  _human_hand_pose_sub;
		float lh_x = 0.0;
		float lh_y = 0.0;
		float lh_z = 0.0;

    };

    visual_plugin::visual_plugin(): line(NULL)
    {
    }

    visual_plugin::~visual_plugin()
    {
    }


	void visual_plugin::leftHandCb(geometry_msgs::Vector3 msg ) {

		lh_x = msg.x;
		lh_y = msg.y;
		lh_z = msg.z;
		
	}
    void visual_plugin::Load( VisualPtr _parent, sdf::ElementPtr /*_sdf*/ )
    {

      if (!ros::isInitialized())
      {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
          return;
      }
      ROS_INFO("Visual Plugin");
	  _node_handle = new ros::NodeHandle();
	  _human_hand_pose_sub = _node_handle->subscribe("/human/hand_pose", 1, &visual_plugin::leftHandCb, this);

      this->visual_ = _parent;
   
      this->visual_namespace_ = "visual/";
      
      std::cout << "SCAAAAAAAAAAAALE: " << visual_->DerivedScale() << std::endl;

      //this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_LIST );
      this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP );


      // init is not allways succesfull here, if visual plugin is loaded at the beginning 
      x =x_offset +1/std::sqrt(2)*r_offset;
      y =y_offset +1/std::sqrt(2)*r_offset;
      z =z_offset;
      poles = ignition::math::Vector3d(5,5,10);
      Init();

	  //physics::WorldPtr world = physics::get_world("default");
      /*
	  actor_model = world->ModelByName("actor");
	  
      //ignition::math::Vector3d link_pose = (ignition::math::Vector3d(x,y,z)-visual_->Pose().Pos())*1000;   
      //ignition::math::Vector3d endpoint = (visual_->WorldPose().Rot().RotateVectorReverse(poles-visual_->WorldPose().Pos()))*1000 ;
	  cout << "LeftHand: " << actor_model->GetLink( "LeftHand")->WorldPose().Pos().X() << " " <<
	  						  actor_model->GetLink( "LeftHand")->WorldPose().Pos().Y() << " "  <<
	  						  actor_model->GetLink( "LeftHand")->WorldPose().Pos().Z() << endl;
	  */
	  //ignition::math::Vector3d pp( spider->GetLink("pole_"+corner_nr)->WorldPose().Pos().X(), spider->GetLink("pole_"+corner_nr)->WorldPose().Pos().Y(), spider->GetLink("pole_"+corner_nr)->WorldPose().Pos().Z()*2);
	  ignition::math::Vector3d link_pose = ignition::math::Vector3d(0, 0, 1);   
	  ignition::math::Vector3d endpoint = ignition::math::Vector3d(3, 0, 1);   
		
      ignition::math::Color color(1,1,1,1);
      line->AddPoint(link_pose,color );
      line->AddPoint(endpoint,color);
      line->SetColor(0,color);
      line->setVisibilityFlags(GZ_VISIBILITY_GUI);
      visual_->SetVisible(true);
      line->Update();

     this->update_connection_ =event::Events:: ConnectPreRender(std::bind(&visual_plugin::UpdateChild, this));
     //update_connection_ =event::Events:: ConnectPostRender(std::bind(&visual_plugin::UpdateChild, this));
    }

    void visual_plugin::UpdateChild()
    {
      if (!init)
      {
        Init();
      }
      //1000 is the scale of the visuale
 
      
      //ignition::math::Vector3d link_pose = (ignition::math::Vector3d(x,y,z)- visual_->Pose().Pos())*1000;  
      //ignition::math::Vector3d  endpoint = (visual_->WorldPose().Rot().RotateVectorReverse(poles- visual_->WorldPose().Pos()))*1000;

	  xline = 1;
	  ignition::math::Vector3d link_pose = ignition::math::Vector3d(0, 0, 0);   
    ignition::math::Vector3d handpoint = ignition::math::Vector3d(lh_x, lh_y, lh_z);  
	  ignition::math::Vector3d endpoint = (visual_->WorldPose().Rot().RotateVectorReverse(handpoint- visual_->WorldPose().Pos()));   
		

      line->SetPoint(0,link_pose);
      line->SetPoint(1,endpoint);
      //this->line->Update();
 
    }

    void visual_plugin::Init() {
	    init = true;
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(visual_plugin)
  }
}