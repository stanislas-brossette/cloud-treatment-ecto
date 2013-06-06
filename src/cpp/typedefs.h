#ifndef TYPEDEFS_H
#define TYPEDEFS_H

typedef boost::variant<
	std::vector<pcl::PlanarRegion<pcl::PointXYZ>,
		Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > >,
	std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>,
		Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB> > >,
	std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>,
		Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > >,
	std::vector<pcl::PlanarRegion<pcl::PointXYZRGBNormal>,
		Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBNormal> > >,
	std::vector<pcl::PlanarRegion<pcl::PointXYZI>,
		Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZI> > >,
	std::vector<pcl::PlanarRegion<pcl::PointNormal>,
		Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointNormal> > >
	> planarRegions_t;

#endif // TYPEDEFS_H
