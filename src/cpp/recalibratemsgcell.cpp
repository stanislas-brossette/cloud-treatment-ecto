#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

#include <ecto/ecto.hpp>

#include <rosbuild_msgs/RecalibrateEnv.h>

using ecto::tendrils;
namespace cloud_treatment
{
	struct RecalibrateMsgCell
	{

		static void declare_params(tendrils& params)
		{
      std::size_t choice_index;
			params.declare<std::size_t> ("choice_index", "Item of the list to publish", choice_index);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
      inputs.declare<std::vector<Eigen::Matrix3f > > ("frames", "list of frames");
      inputs.declare<std::vector<Eigen::Vector4f > > ("origins", "list of origins");
      outputs.declare<boost::shared_ptr<geometry_msgs::PoseStamped const> > (
                                                                      "pose_stamped_msg", 
                                                                      "pose message to publish");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
      origins_ = inputs["origins"];
      frames_ = inputs["frames"];
      pose_stamped_msg_ = outputs["pose_stamped_msg"];
      choice_index_ = params["choice_index"];
		}


		int process(const tendrils& inputs, const tendrils& outputs)
		{
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.header.frame_id = "world";
      poseStamped.pose.position.x = origins_->at(*choice_index_).x();
      poseStamped.pose.position.y = origins_->at(*choice_index_).y();
      poseStamped.pose.position.z = origins_->at(*choice_index_).z();
      Eigen::Quaternionf quaternion (frames_->at(*choice_index_));
      poseStamped.pose.orientation.x = quaternion.x(); 
      poseStamped.pose.orientation.y = quaternion.y(); 
      poseStamped.pose.orientation.z = quaternion.z(); 
      poseStamped.pose.orientation.w = quaternion.w(); 
      *pose_stamped_msg_ = 
               boost::make_shared<geometry_msgs::PoseStamped const>(poseStamped); 
      std::cout << "position:\n" << (*pose_stamped_msg_)->pose.position << std::endl;
      std::cout << "orientation:\n" <<(*pose_stamped_msg_)->pose.orientation << std::endl;
			return ecto::OK;
		}
    ecto::spore< boost::shared_ptr<geometry_msgs::PoseStamped const> > pose_stamped_msg_;
    ecto::spore< std::size_t > choice_index_;
    ecto::spore< std::string > topic_name_;
    ecto::spore< std::vector< Eigen::Vector4f > > origins_;
    ecto::spore< std::vector< Eigen::Matrix3f > > frames_; 
	};
}

ECTO_CELL(cloud_treatment, cloud_treatment::RecalibrateMsgCell,
		  "RecalibrateMsgCell", "Publishes a ros message with origin and frame that can be used for recalibration");

