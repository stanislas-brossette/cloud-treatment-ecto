#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

namespace cloud_treatment
{
	struct NormalSegmentationCell
	{
		static void declare_params(tendrils& params)
		{
			double angle_threshold = 0;
			double curvature_threshold = 0;

			params.declare<double> ("angle_threshold",
									"Angle threshold for segmentation.", angle_threshold);
			params.declare<double> ("curvature_threshold",
									"Angle threshold for segmentation.", curvature_threshold);
		}


		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::PointCloud> ("output", "Filtered Cloud.");
		}


		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
			angle_threshold_ = params["angle_threshold"];
			curvature_threshold_ = params["curvature_threshold"];

			output_ = outputs["output"];
		}


		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
		{
			typename ::pcl::PointCloud<Point>::Ptr filteredCloud(new typename ::pcl::PointCloud<Point>);
            std::cout << "This cell shouldn't be called with a cloud that doesn't have normals" << std::endl;

            for(std::size_t i = 0; i < input->size(); ++i)
            {
                filteredCloud->push_back(input->points[i]);
            }

			*output_ = ecto::pcl::xyz_cloud_variant_t(filteredCloud);

			return ecto::OK;
		}

		
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const ::pcl::PointCloud<pcl::PointXYZRGBNormal> >& inputCloud)
		{
			typename ::pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filteredCloud(new typename ::pcl::PointCloud<pcl::PointXYZRGBNormal>);
			typename ::pcl::PointCloud<pcl::Normal>::Ptr normalsCloud(new typename ::pcl::PointCloud<pcl::Normal>);
			normalsCloud->resize(inputCloud->size());

			for(std::size_t i = 0; i < inputCloud->size(); ++i)
			{
				normalsCloud->at(i) = ::pcl::Normal(inputCloud->points[i].normal_x,
													inputCloud->points[i].normal_y,
													inputCloud->points[i].normal_z);
			}

			pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal> > (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

//			pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
//			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//			normal_estimator.setSearchMethod (tree);
//			normal_estimator.setInputCloud (cloud);
//			normal_estimator.setKSearch (50);
//			normal_estimator.compute (*normals);

			pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg;
			reg.setMinClusterSize (1000);
			reg.setMaxClusterSize (10000000);
			reg.setSearchMethod (tree);
			reg.setNumberOfNeighbours (30);
			reg.setInputCloud (inputCloud);
			//reg.setIndices (indices);
			reg.setInputNormals (normalsCloud);
			reg.setSmoothnessThreshold (7.0 / 180.0 * M_PI);
			reg.setCurvatureThreshold (1.0);

			std::vector <pcl::PointIndices> clusters;

			reg.extract (clusters);

			std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

			pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

			*output_ = ecto::pcl::xyz_cloud_variant_t(colored_cloud);

			return ecto::OK;
		}

		ecto::spore<double> angle_threshold_;
		ecto::spore<double> curvature_threshold_;
		ecto::spore<ecto::pcl::PointCloud> output_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCell<cloud_treatment::NormalSegmentationCell>,
		  "NormalSegmentationCell", "Normal Segmentation");

