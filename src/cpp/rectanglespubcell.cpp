#include <boost/make_shared.hpp>

#include <Eigen/Core>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <ecto/ecto.hpp>

using ecto::tendrils;
namespace cloud_treatment
{
	struct RectanglesPubCell
	{
		static void declare_params(tendrils& params)
		{
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
      inputs.declare<std::vector< std::vector<Eigen::Vector3f> > > (
                                     "rectangles", "rectangles");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped const> > (
                                     "rectanglemsg0", "rectanglemsg0");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped > > (
                                     "rectanglemsg1", "rectanglemsg1");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped > > (
                                     "rectanglemsg2", "rectanglemsg2");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped > > (
                                     "rectanglemsg3", "rectanglemsg3");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped > > (
                                     "rectanglemsg4", "rectanglemsg4");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped > > (
                                     "rectanglemsg5", "rectanglemsg5");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped > > (
                                     "rectanglemsg6", "rectanglemsg6");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped > > (
                                     "rectanglemsg7", "rectanglemsg7");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped > > (
                                     "rectanglemsg8", "rectanglemsg8");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped > > (
                                     "rectanglemsg9", "rectanglemsg9");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
      rectangles_ = inputs["rectangles"];
			rectanglemsg0_ = outputs["rectanglemsg0"];
			rectanglemsg1_ = outputs["rectanglemsg1"];
			rectanglemsg2_ = outputs["rectanglemsg2"];
			rectanglemsg3_ = outputs["rectanglemsg3"];
			rectanglemsg4_ = outputs["rectanglemsg4"];
			rectanglemsg5_ = outputs["rectanglemsg5"];
			rectanglemsg6_ = outputs["rectanglemsg6"];
			rectanglemsg7_ = outputs["rectanglemsg7"];
			rectanglemsg8_ = outputs["rectanglemsg8"];
			rectanglemsg9_ = outputs["rectanglemsg9"];
		}

		int process(const tendrils& inputs, const tendrils& outputs)
		{
      geometry_msgs::PolygonStamped polygon;
      std::cout << "rectangle test " << rectangles_->at(0)[0].x() << std::endl; 
      polygon.header.frame_id = "/world";
      geometry_msgs::Point32 point0;
      geometry_msgs::Point32 point1;
      geometry_msgs::Point32 point2;
      geometry_msgs::Point32 point3;
      point0.x = 0.0;
      point0.y = 0.0;
      point0.z = 0.0;
      point1.x = 1.0;
      point1.y = 0.0;
      point1.z = 0.0;
      point2.x = 1.0;
      point2.y = 1.0;
      point2.z = 0.0;
      point3.x = 0.0;
      point3.y = 1.0;
      point3.z = 0.0;
      polygon.polygon.points.push_back(point0);
      polygon.polygon.points.push_back(point1);
      polygon.polygon.points.push_back(point2);
      polygon.polygon.points.push_back(point3);
      polygon.polygon.points.push_back(point0);
      *rectanglemsg0_ = boost::make_shared<geometry_msgs::PolygonStamped const>(polygon);
      std::cout << (*rectanglemsg0_)->polygon.points.size();
			return ecto::OK;
		}
    ecto::spore< std::vector< std::vector<Eigen::Vector3f> > > rectangles_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg0_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped > > rectanglemsg1_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped > > rectanglemsg2_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped > > rectanglemsg3_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped > > rectanglemsg4_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped > > rectanglemsg5_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped > > rectanglemsg6_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped > > rectanglemsg7_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped > > rectanglemsg8_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped > > rectanglemsg9_;
	};
}

ECTO_CELL(cloud_treatment, cloud_treatment::RectanglesPubCell,
		  "RectanglesPubCell", "Transforms a vector of rectangles into several polygon msgs");

