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

#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>

#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>
#include <pcl/io/pcd_io.h>

#include "dirs.hh"

namespace cloud_treatment
{
	struct PCDReaderCell
	{
		PCDReaderCell() { first = true; }

		static void declare_params(tendrils& params)
		{
			params.declare<ecto::pcl::Format>("format", "Format of cloud found in PCD file.",
											  ecto::pcl::FORMAT_XYZRGBNORMAL);
			params.declare<std::string> ("filename", "Name of the pcd file", "");
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::PointCloud>("output", "A point cloud from the pcd file.");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
			output_ = outputs["output"];
			format_ = params["format"];
			filename_ = params["filename"];
		}

		int process(const tendrils& /*inputs*/, const tendrils& outputs)
		{
			if (!first)
				return ecto::OK;
			first = false;
			std::string filePath = getCloudFilePath(*filename_);
			switch(*format_)
			{
			case ecto::pcl::FORMAT_XYZ:
			{
				::pcl::PointCloud< ::pcl::PointXYZ >::Ptr cloud (
							new ::pcl::PointCloud< ::pcl::PointXYZ >);
				if ( ::pcl::io::loadPCDFile< ::pcl::PointXYZ >(filePath, *cloud) == -1)
				{
					throw std::runtime_error("PCDReaderCell: failed to read PointXYZ cloud.");
					return 1;
				}
				std::cout << "Made it this far" << std::endl;
				ecto::pcl::PointCloud p( cloud );
				*output_ = p;
			} break;
			case ecto::pcl::FORMAT_XYZRGB:
			{
				::pcl::PointCloud< ::pcl::PointXYZRGB >::Ptr cloud (
							new ::pcl::PointCloud< ::pcl::PointXYZRGB >);
				if ( ::pcl::io::loadPCDFile< ::pcl::PointXYZRGB > (filePath, *cloud) == -1)
				{
					throw std::runtime_error("PCDReaderCell: failed to read PointXYZRGB cloud.");
					return 1;
				}
				ecto::pcl::PointCloud p( cloud );
				*output_ = p;
			} break;
			case ecto::pcl::FORMAT_XYZRGBNORMAL:
			{
				::pcl::PointCloud< ::pcl::PointXYZRGBNormal >::Ptr cloud (
							new ::pcl::PointCloud< ::pcl::PointXYZRGBNormal >);
				if ( ::pcl::io::loadPCDFile< ::pcl::PointXYZRGBNormal > (filePath, *cloud) == -1)
				{
					throw std::runtime_error("PCDReaderCell: failed to read PointXYZRGBNormal cloud.");
					return 1;
				}
				ecto::pcl::PointCloud p( cloud );
				*output_ = p;
			} break;
			default:
				throw std::runtime_error("PCDReaderCell: Unknown cloud type.");
			}
			return ecto::OK;
		}

		std::string getCloudFilePath(std::string filename)
		{
			namespace fs = boost::filesystem;

			fs::path pointCloudPath (filename);
			fs::path pointCloudPathInstall (POINT_CLOUD_PATH);
			pointCloudPathInstall /= filename;
			fs::path pointCloudPathBuild (POINT_CLOUD_BUILD_PATH);
			pointCloudPathBuild /= filename;

			if (fs::is_regular_file (pointCloudPath))
			{
			}
			else if (fs::is_regular_file (pointCloudPathBuild))
				pointCloudPath = pointCloudPathBuild;
			else if (fs::is_regular_file (pointCloudPathInstall))
				pointCloudPath = pointCloudPathInstall;
			else
			{
				throw std::runtime_error((boost::format (
											"point cloud file \"%1%\" does not exist")
											% filename).str ());
			}
			return pointCloudPath.string();
		}

		bool first;
		ecto::spore<ecto::pcl::PointCloud> output_;
		ecto::spore<ecto::pcl::Format> format_;
		ecto::spore<std::string> filename_;

	};

}

ECTO_CELL(cloud_treatment, cloud_treatment::PCDReaderCell, "PCDReaderCell", "Read a cloud from a PCD file");
