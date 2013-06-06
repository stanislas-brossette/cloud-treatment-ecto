#include <boost/make_shared.hpp>
#include <boost/variant.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

namespace cloud_treatment
{
	struct OrganizedMultiPlaneSegmentationCell
	{
//		typedef std::vector<pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBNormal> > >  planar_regions;

		static void declare_params(ecto::tendrils& params)
		{
			pcl::OrganizedMultiPlaneSegmentation<
					pcl::PointXYZ, pcl::Normal, pcl::Label> default_;
			unsigned plane_min_inliers = default_.getMinInliers();
			double plane_angular_threshold = default_.getAngularThreshold();
			double plane_distance_threshold = default_.getDistanceThreshold();
			double maximum_curvature = default_.getMaximumCurvature();
			bool use_planar_refinement = false;

			////TODO////
			//bool 	project_points_
			//Whether or not points should be projected to the plane,
			//or left in the original 3D space.
			//PlaneComparatorPtr 	compare_
			//A comparator for comparing neighboring pixels' plane equations.
			//PlaneRefinementComparatorPtr 	refinement_compare_
			//A comparator for use on the refinement step.
			////////////

			params.declare<unsigned> ("plane_min_inliers",
								 "The minimum number of inliers required for each plane",
								 plane_min_inliers);
			params.declare<double> ("plane_angular_threshold",
								 "The tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane",
								 plane_angular_threshold);
			params.declare<double> ("plane_distance_threshold",
								 "The tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane",
								 plane_distance_threshold);
			params.declare<double> ("maximum_curvature",
								 "The tolerance for maximum curvature after fitting a plane",
								 maximum_curvature);
			params.declare<bool> ("use_planar_refinement",
								 "use planar refinement",
								 use_planar_refinement);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::PointCloud> ("output", "Filtered Cloud.");
			outputs.declare<ecto::pcl::PointCloud> ("output_bis_", "Filtered Cloud.");
//			outputs.declare< planar_regions > ("regions", "Segmented plane regions");
		}

		void configure(const tendrils& params, const tendrils& inputs,
					   const tendrils& outputs)
		{
			plane_min_inliers_ = params["plane_min_inliers"];
			plane_angular_threshold_ = params["plane_angular_threshold"];
			plane_distance_threshold_ = params["plane_distance_threshold"];
			maximum_curvature_ = params["maximum_curvature"];
			use_planar_refinement_ = params["use_planar_refinement"];

//			regions_ = outputs["regions"];
			output_ = outputs["output"];
			output_bis_ = outputs["output_bis_"];
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const pcl::PointCloud<Point> >& input,
					boost::shared_ptr<const pcl::PointCloud<pcl::Normal > >& normals)
		{
			pcl::OrganizedMultiPlaneSegmentation<Point, pcl::Normal, pcl::Label> mps;

			//Set the algorithm's parameters
			mps.setMinInliers (*plane_min_inliers_);
			mps.setAngularThreshold (pcl::deg2rad (*plane_angular_threshold_)); //3 degrees
			mps.setDistanceThreshold (*plane_distance_threshold_); //2cm
			mps.setMaximumCurvature (*maximum_curvature_);

			std::vector<pcl::PlanarRegion<Point>,
					Eigen::aligned_allocator<pcl::PlanarRegion<Point> > > regions;
			std::vector<pcl::ModelCoefficients> model_coefficients;
			std::vector<pcl::PointIndices> inlier_indices;
			pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
			std::vector<pcl::PointIndices> label_indices;
			std::vector<pcl::PointIndices> boundary_indices;
			mps.setInputNormals (normals);
			mps.setInputCloud (input);

			if (*use_planar_refinement_)
			{
				std::cout << "Using planar refinement\n";
				mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
			}
			else
			{
				std::cout << "Not using planar refinement\n";
				mps.segment (regions);//, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
			}


			boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_ = boost::make_shared<pcl::visualization::PCLVisualizer> ("3D Viewer");
			vis_->initCameraParameters ();
			vis_->addPointCloud<Point> (input, "cloudname");
			vis_->resetCameraViewpoint ("cloudname");

			char name[1024];
			unsigned char red [6] = {255,   0,   0, 255, 255,   0};
			unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
			unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

			typename pcl::PointCloud<Point>::Ptr contour (new pcl::PointCloud<Point>);

			for (size_t i = 0; i < regions.size (); i++)
			{
				Eigen::Vector3f centroid = regions[i].getCentroid ();
				Eigen::Vector4f model = regions[i].getCoefficients ();
				pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
				pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
												   centroid[1] + (0.5f * model[1]),
												   centroid[2] + (0.5f * model[2]));
				sprintf (name, "normal_%d", unsigned (i));
				vis_->addArrow (pt2, pt1, 1.0, 0, 0, false, name);

				contour->points = regions[i].getContour ();
				sprintf (name, "plane_%02d", int (i));
				pcl::visualization::PointCloudColorHandlerCustom <Point> color (contour, red[i%6], grn[i%6], blu[i%6]);
				if(!vis_->updatePointCloud(contour, color, name))
					vis_->addPointCloud (contour, color, name);
				vis_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
			}


			while (!vis_->wasStopped ())
			{
				vis_->spinOnce (100);
				boost::this_thread::sleep (boost::posix_time::microseconds (1000));
			}

//			*regions_ = regions;
			*output_ = ecto::pcl::xyz_cloud_variant_t(input);
			*output_bis_ = ecto::pcl::xyz_cloud_variant_t(input);

			return ecto::OK;
		}
		// Store params/inputs/outputs in spores
		ecto::spore<unsigned> plane_min_inliers_;
		ecto::spore<double> plane_angular_threshold_;
		ecto::spore<double> plane_distance_threshold_;
		ecto::spore<double> maximum_curvature_;
		ecto::spore<bool> use_planar_refinement_;

//		ecto::spore<planar_regions> regions_;
		ecto::spore<ecto::pcl::PointCloud> output_;
		ecto::spore<ecto::pcl::PointCloud> output_bis_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCellWithNormals<cloud_treatment::OrganizedMultiPlaneSegmentationCell>,
		  "OrganizedMultiPlaneSegmentationCell",
		  "Organized multi plane segmentation");
