#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

namespace cloud_treatment
{
	struct XYZSwitchCell
	{
		static void declare_params(ecto::tendrils& params)
		{
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::PointCloud> ("output", "XYZ switched cloud");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
			output_ = outputs["output"];
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const pcl::PointCloud<Point> >& input)
		{
			typename pcl::PointCloud<Point>::Ptr cloud(new typename pcl::PointCloud<Point>);

			float x, y, z;
			cloud->points.resize(input->points.size());
			for(std::size_t i = 0; i<input->points.size (); ++i)
			{
				cloud->points[i] = input->points[i];
				x = input->points[i].z;
				y = -input->points[i].x;
				z = -input->points[i].y;

				cloud->points[i].x = x;
				cloud->points[i].y = y;
				cloud->points[i].z = z;
			}

			*output_ = ecto::pcl::xyz_cloud_variant_t(cloud);
			return ecto::OK;
		}
		ecto::spore<ecto::pcl::PointCloud> output_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCell<cloud_treatment::XYZSwitchCell>, "XYZSwitchCell",
		  "Switches the x, y and z values of the point cloud to fit with AMELIF's frame");
