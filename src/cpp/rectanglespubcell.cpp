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
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped const> > (
                                     "rectanglemsg1", "rectanglemsg1");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped const> > (
                                     "rectanglemsg2", "rectanglemsg2");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped const> > (
                                     "rectanglemsg3", "rectanglemsg3");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped const> > (
                                     "rectanglemsg4", "rectanglemsg4");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped const> > (
                                     "rectanglemsg5", "rectanglemsg5");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped const> > (
                                     "rectanglemsg6", "rectanglemsg6");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped const> > (
                                     "rectanglemsg7", "rectanglemsg7");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped const> > (
                                     "rectanglemsg8", "rectanglemsg8");
			outputs.declare<boost::shared_ptr<geometry_msgs::PolygonStamped const> > (
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

     boost::shared_ptr<geometry_msgs::PolygonStamped const> fillPolygonStamped(
                           const std::vector<Eigen::Vector3f>& rectangle)
    {
      geometry_msgs::PolygonStamped Polygon;
      Polygon.header.frame_id = "/world";
      geometry_msgs::Point32 point;
      for(std::size_t i = 0; i < rectangle.size()+1; ++i)
      {
        point.x = rectangle[i % rectangle.size()].x();
        point.y = rectangle[i % rectangle.size()].y();
        point.z = rectangle[i % rectangle.size()].z();
        Polygon.polygon.points.push_back(point);
      }
      return boost::make_shared<geometry_msgs::PolygonStamped const>(Polygon);
    }

		int process(const tendrils& inputs, const tendrils& outputs)
		{
      *rectanglemsg0_ = fillPolygonStamped(rectangles_->at(0));      
      std::cout << (*rectanglemsg0_)->polygon.points.size();
			return ecto::OK;
		}
    ecto::spore< std::vector< std::vector<Eigen::Vector3f> > > rectangles_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg0_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg1_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg2_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg3_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg4_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg5_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg6_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg7_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg8_;
    ecto::spore< boost::shared_ptr<geometry_msgs::PolygonStamped const> > rectanglemsg9_;
	};
}

ECTO_CELL(cloud_treatment, cloud_treatment::RectanglesPubCell,
		  "RectanglesPubCell", "Transforms a vector of rectangles into several polygon msgs");

