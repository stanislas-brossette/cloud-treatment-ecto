#include <pcl/filters/passthrough.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

namespace cloud_treatment
{
	struct PassThrough3DCell
	{
		static void declare_params(tendrils& params)
		{
			::pcl::PassThrough< ::pcl::PointXYZ > default_;
			params.declare<bool> ("keep_organized",
					"To keep the point-cloud organized or not", false);
	#if PCL_VERSION_COMPARE(<,1,6,0)
			double x_min = -100;
			double x_max = 100;
			double y_min= -100;
			double y_max = 100;
			double z_min = -100;
			double z_max = 100;
			//default_.getFilterLimits(x_min, x_max);
			//default_.getFilterLimits(y_min, y_max);
			//default_.getFilterLimits(z_min, z_max);
	#else
			float x_min = -100;
			float x_max = 100;
			float y_min= -100;
			float y_max = 100;
			float z_min = -100;
			float z_max = 100;
			//default_.getFilterLimits(x_min, x_max);
			//default_.getFilterLimits(y_min, y_max);
			//default_.getFilterLimits(z_min, z_max);
	#endif
			params.declare<double> ("x_min",
					"Minimum x value for the filter.", x_min);
			params.declare<double> ("x_max",
					"Minimum x value for the filter.", x_max);
			params.declare<double> ("y_min",
					"Minimum y value for the filter.", y_min);
			params.declare<double> ("y_max",
					"Minimum y value for the filter.", y_max);
			params.declare<double> ("z_min",
					"Minimum z value for the filter.", z_min);
			params.declare<double> ("z_max",
					"Minimum z value for the filter.", z_max);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::PointCloud> ("output", "Filtered Cloud.");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
			x_min_ = params["x_min"];
			x_max_ = params["x_max"];
			y_min_ = params["y_min"];
			y_max_ = params["y_max"];
			z_min_ = params["z_min"];
			z_max_ = params["z_max"];
			keep_organized_ = params["keep_organized"];

			output_ = outputs["output"];
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
		{
      //filter along X direction
      ::pcl::PassThrough<Point> filterX;
			filterX.setFilterFieldName("x");
			filterX.setFilterLimits(*x_min_, *x_max_);
			//filterX.setKeepOrganized(keep_organized_);
			filterX.setInputCloud(input);
			typename ::pcl::PointCloud<Point>::Ptr cloudX(new typename ::pcl::PointCloud<Point>);
			filterX.filter(*cloudX);
			cloudX->header = input->header;

      //filter along Y direction
      ::pcl::PassThrough<Point> filterY;
			filterY.setFilterFieldName("y");
			filterY.setFilterLimits(*y_min_, *y_max_);
			//filterY.setKeepOrganized(keep_organized_);
			filterY.setInputCloud(cloudX);
			typename ::pcl::PointCloud<Point>::Ptr cloudXY(new typename ::pcl::PointCloud<Point>);
			filterY.filter(*cloudXY);
			cloudXY->header = input->header;

      //filter along Z direction
      ::pcl::PassThrough<Point> filterZ;
			filterZ.setFilterFieldName("z");
			filterZ.setFilterLimits(*z_min_, *z_max_);
			//filterX.setKeepOrganized(keep_organized_);
			filterZ.setInputCloud(cloudXY);
			typename ::pcl::PointCloud<Point>::Ptr cloudXYZ(new typename ::pcl::PointCloud<Point>);
			filterZ.filter(*cloudXYZ);
			cloudXYZ->header = input->header;

			*output_ = ecto::pcl::xyz_cloud_variant_t(cloudXYZ);

			return ecto::OK;
		}

		ecto::spore<double> x_min_;
		ecto::spore<double> x_max_;
		ecto::spore<double> y_min_;
		ecto::spore<double> y_max_;
		ecto::spore<double> z_min_;
		ecto::spore<double> z_max_;
		ecto::spore<bool> keep_organized_;
		ecto::spore<ecto::pcl::PointCloud> output_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCell<cloud_treatment::PassThrough3DCell>,
		  "PassThrough3DCell", "3D PassThrough filter");

