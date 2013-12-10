/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <Eigen/Core>

#include <boost/variant/get.hpp>
#include <boost/format.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ecto_pcl/ecto_pcl.hpp>

#include "typedefs.h"

namespace cloud_treatment
{
	struct CloudViewerCell
	{

		CloudViewerCell()
			:
			  quit(false)
		{
		}

		static void
		declare_params(tendrils& params)
		{
			params.declare<std::string>("window_name", "The window name",
										"cloud viewer");
		}

		static void
		declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			inputs.declare<ecto::pcl::PointCloud>("input", "The cloud to view");
			inputs.declare< planarRegions_t > (
						"regions", "Segmented plane regions").required(false);
			inputs.declare< std::vector < Eigen::Vector4f> > (
						"VIPoints", "List of important points to display in big red").required(false);
			inputs.declare<std::vector< std::vector<Eigen::Vector3f> > > (
						"rectangles", "List of rectangles to plot ").required(false);
		}

		void
		configure(const tendrils& params, const tendrils& inputs,
				  const tendrils& outputs)
		{
			params["window_name"] >> window_name;
      spheresCreated_ = 0;
		}
		void
		run()
		{
			quit = false;
			viewer_.reset(new pcl::visualization::PCLVisualizer(window_name));
			viewer_->setBackgroundColor(0, 0, 0);
//			viewer_->addCoordinateSystem(0.25);
			viewer_->initCameraParameters();

			while (!viewer_->wasStopped()/* &&
				   !boost::this_thread::interruption_requested()*/)
			{
				{
					boost::mutex::scoped_try_lock lock(mtx);
					if (lock)
					{
						signal_();
						jobs_.clear(); //disconnects all the slots.
					}
				}
				viewer_->spinOnce(20);
			}
			quit = true;
		}

		struct show_dispatch: boost::static_visitor<>
		{
			show_dispatch(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
						  const std::string& key)
				:
				  viewer(viewer),
				  key(key)
			{
			}

			template<typename Point>
			void
			operator()(boost::shared_ptr<const ::pcl::PointCloud<Point> >& cloud) const
			{
				std::cout << "Cloud->size() = " << cloud->size() << std::endl;
				if (!viewer->updatePointCloud<Point>(cloud, key))
				{
					viewer->addPointCloud<Point>(cloud, key);
//					viewer->resetCameraViewpoint (key);
				}
			}
			void
			operator()(boost::shared_ptr<const CloudPOINTXYZRGB>& cloud) const
			{
				std::cout << "CloudPOINTXYZRGB->size() = " << cloud->size() << std::endl;
				::pcl::visualization::PointCloudColorHandlerRGBField<
						CloudPOINTXYZRGB::PointType> rgb(cloud);
				if (!viewer->updatePointCloud(cloud, rgb, key))
				{
					viewer->addPointCloud(cloud, rgb, key);
//					viewer->resetCameraViewpoint (key);
				}
			}
			void
			operator()(boost::shared_ptr<const CloudPOINTXYZRGBNORMAL>& cloud) const
			{
				std::cout << "CloudPOINTXYZRGBNORMAL->size() = " << cloud->size() <<
							 std::endl;
				::pcl::visualization::PointCloudColorHandlerRGBField<
						CloudPOINTXYZRGBNORMAL::PointType> rgb(cloud);
				//::pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<
				//      CloudPOINTXYZRGBNORMAL::PointType> normals(cloud);

				if (!viewer->updatePointCloud(cloud, rgb, key))
				{
					viewer->addPointCloud(cloud, rgb, key);
//					viewer->resetCameraViewpoint (key);
				}
				//          viewer->updatePointCloud(cloud,normals,key);
			}


			template <typename T>
			void operator()(std::vector<pcl::PlanarRegion<T>,
					Eigen::aligned_allocator<pcl::PlanarRegion<T> > > & regions) const
			{
				char name[1024];
				unsigned char red [6] = {255,   0,   0, 255, 255,   0};
				unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
				unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

				typename pcl::PointCloud<T>::Ptr contour (
							new pcl::PointCloud<T>);

				for (size_t i = 0; i < regions.size (); i++)
				{
					Eigen::Vector3f centroid = regions.at(i).getCentroid ();
					Eigen::Vector4f model = regions.at(i).getCoefficients ();
					pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
					pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
													   centroid[1] + (0.5f * model[1]),
													   centroid[2] + (0.5f * model[2]));
					sprintf (name, "normal_region_%d", unsigned (i));
					viewer->removeShape(name);
					viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);

					contour->points = regions.at(i).getContour ();
					sprintf (name, "plane_region_%02d", int (i));
					pcl::visualization::PointCloudColorHandlerCustom <T> color (
								contour, red[i%6], grn[i%6], blu[i%6]);
					if(!viewer->updatePointCloud(contour, color, name))
						viewer->addPointCloud (contour, color, name);
					viewer->setPointCloudRenderingProperties (
								pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
				}
			}

			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
			std::string key;
		};
		struct show_dispatch_runner
		{
			show_dispatch_runner(const show_dispatch& dispatch,
								 const ecto::pcl::xyz_cloud_variant_t& varient,
								 const planarRegions_t planarRegions)
				:
				  dispatch_(dispatch),
				  varient_(varient),
				  planarRegions_(planarRegions)
			{
			}
			void
			operator()()
			{
				boost::apply_visitor(dispatch_, varient_);
				boost::apply_visitor(dispatch_, planarRegions_);
			}
			show_dispatch dispatch_;
			ecto::pcl::xyz_cloud_variant_t varient_;
			planarRegions_t planarRegions_;
		};

		int
		process(const tendrils& inputs, const tendrils& outputs)
		{
			if (quit)
			{
				runner_thread_->join();
				return ecto::QUIT;
			}
			if (!runner_thread_)
			{
				runner_thread_.reset(new boost::thread(boost::bind(
														   &CloudViewerCell::run, this)));
			}
			while (!viewer_)
			{
				boost::this_thread::sleep(boost::posix_time::milliseconds(10));
			}

			{
				boost::mutex::scoped_lock lock(mtx);
				ecto::pcl::PointCloud cloud = inputs.get<ecto::pcl::PointCloud>("input");
				ecto::pcl::xyz_cloud_variant_t varient = cloud.make_variant();

				planarRegions_t planarRegions = inputs.get<planarRegions_t>("regions");

        std::vector < Eigen::Vector4f> VIPoints; 
        VIPoints = inputs.get<std::vector < Eigen::Vector4f> > ("VIPoints");
        std::vector< std::vector<Eigen::Vector3f> > rectangles_; 
        rectangles_ = 
          inputs.get< std::vector< std::vector<Eigen::Vector3f> > > ("rectangles");
        
        //Add the VIPoints as spheres
        boost::format fmt("sphere%d");
        for(std::size_t i = 0; i < VIPoints.size(); ++i)
        {
          ::pcl::PointXYZ sphereCenter = ::pcl::PointXYZ(VIPoints[i].x(),
                                                         VIPoints[i].y(),
                                                         VIPoints[i].z());
          fmt % i;
          if( i+1 > spheresCreated_)
          {
            viewer_->addSphere(sphereCenter, 0.015, 255, 0, 0, fmt.str());
            spheresCreated_ = i+1;
          }
          else
            viewer_->updateSphere(sphereCenter, 0.01, 255, 0, 0, fmt.str());
        }
        //Add the rectangles representing the steps 
        boost::format fmtline("line%d %d");
        for (std::size_t i = 0; i < rectangles_.size(); ++i)
        {
          if(rectanglesCreated_.size() < i+1 || rectanglesMemory_[i] != rectangles_[i][0].x())
          {
            if(rectanglesCreated_.size() < i+1)
            {
              rectanglesCreated_.push_back(false);
              rectanglesMemory_.push_back(99999.);
            }  
            rectanglesMemory_[i] = rectangles_[i][0].x();
            ::pcl::PointXYZ point0 = ::pcl::PointXYZ(rectangles_[i][0].x(),
                                                     rectangles_[i][0].y(),
                                                     rectangles_[i][0].z());
            ::pcl::PointXYZ point1 = ::pcl::PointXYZ(rectangles_[i][1].x(),
                                                     rectangles_[i][1].y(),
                                                     rectangles_[i][1].z());
            ::pcl::PointXYZ point2 = ::pcl::PointXYZ(rectangles_[i][2].x(),
                                                     rectangles_[i][2].y(),
                                                     rectangles_[i][2].z());
            ::pcl::PointXYZ point3 = ::pcl::PointXYZ(rectangles_[i][3].x(),
                                                     rectangles_[i][3].y(),
                                                     rectangles_[i][3].z());
            if(rectanglesCreated_[i])
            {
              viewer_->removeShape((fmtline % i % 0).str());
              viewer_->removeShape((fmtline % i % 1).str());
              viewer_->removeShape((fmtline % i % 2).str());
              viewer_->removeShape((fmtline % i % 3).str());
            }
            viewer_->addLine(point0, point1, (fmtline % i % 0).str());
            viewer_->addLine(point1, point2, (fmtline % i % 1).str());
            viewer_->addLine(point2, point3, (fmtline % i % 2).str());
            viewer_->addLine(point3, point0, (fmtline % i % 3).str());
            rectanglesCreated_[i] = true;
          }
        }
				show_dispatch dispatch(viewer_, "main cloud");
				boost::shared_ptr<boost::signals2::scoped_connection> c(
							new boost::signals2::scoped_connection);
				*c = signal_.connect(show_dispatch_runner(dispatch, varient, planarRegions));
				jobs_.push_back(c);
			}

			return 0;
		}

		~CloudViewerCell()
		{
			if (runner_thread_)
			{
				runner_thread_->interrupt();
				runner_thread_->join();
			}
		}
		std::string window_name;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
		boost::shared_ptr<boost::thread> runner_thread_;
		boost::signals2::signal<void
		(void)> signal_;
		std::vector<boost::shared_ptr<boost::signals2::scoped_connection> > jobs_;
		boost::mutex mtx;
		bool quit;
    int spheresCreated_;
    std::vector<bool> rectanglesCreated_;
    std::vector<double> rectanglesMemory_;
	};

}

ECTO_CELL(cloud_treatment, cloud_treatment::CloudViewerCell,
		  "CloudViewerCell", "Viewer of clouds");
