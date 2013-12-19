#include <pcl/filters/passthrough.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

namespace cloud_treatment
{
	struct PassThroughCell
	{
		static void declare_params(tendrils& params)
		{
			::pcl::PassThrough< ::pcl::PointXYZ > default_;
			params.declare<std::string> ("filter_field_name",
										 "The name of the field to use for filtering.", "");
			float filter_limit_min, filter_limit_max;
			default_.getFilterLimits(filter_limit_min, filter_limit_max);

			params.declare<double> ("filter_limit_min",
									"Minimum value for the filter.", filter_limit_min);
			params.declare<double> ("filter_limit_max",
									"Maximum value for the filter.", filter_limit_max);
			params.declare<bool> ("filter_limit_negative",
								  "To negate the limits or not.",
								  default_.getFilterLimitsNegative());
			params.declare<bool> ("keep_organized",
								  "To keep the point-cloud organized or not", true);
		}

		static void declare_io(const tendrils& params, 
                           tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::PointCloud> ("output", "Filtered Cloud.");
		}

		void configure(const tendrils& params, 
                   const tendrils& inputs, const tendrils& outputs)
		{
			filter_field_name_ = params["filter_field_name"];
			filter_limit_min_ = params["filter_limit_min"];
			filter_limit_max_ = params["filter_limit_max"];
			filter_limit_negative_ = params["filter_limit_negative"];
			keep_organized_ = params["keep_organized"];

			output_ = outputs["output"];
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					      boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
		{
			::pcl::PassThrough<Point> filter;
			filter.setFilterFieldName(*filter_field_name_);
			filter.setFilterLimits(*filter_limit_min_, *filter_limit_max_);
			filter.setFilterLimitsNegative(*filter_limit_negative_);
			filter.setKeepOrganized(keep_organized_);
			filter.setInputCloud(input);

			typename ::pcl::PointCloud<Point>::Ptr cloud(
                                     new typename ::pcl::PointCloud<Point>);
			filter.filter(*cloud);
			cloud->header = input->header;
			*output_ = ecto::pcl::xyz_cloud_variant_t(cloud);

			return ecto::OK;
		}

		ecto::spore<std::string> filter_field_name_;
		ecto::spore<double> filter_limit_min_;
		ecto::spore<double> filter_limit_max_;
		ecto::spore<bool> filter_limit_negative_;
		ecto::spore<bool> keep_organized_;
		ecto::spore<ecto::pcl::PointCloud> output_;
	};
}



ECTO_CELL(cloud_treatment,
      ecto::pcl::PclCell<cloud_treatment::PassThroughCell>,
		  "PassThroughCell", "PassThrough filter");

