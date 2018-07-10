// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

#include <pcl/point_types.h>

#include <pcl/common/io.h>

#include <pcl/keypoints/sift_keypoint.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>

//#include <pcl/console/print.cpp> for cross compiling

#include <pcl/impl/pcl_base.hpp>///////////////

#include <pcl/search/pcl_search.h>
#include <pcl/search/impl/search.hpp>///
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/filters/impl/voxel_grid.hpp>

#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/console/parse.h>
/* This example shows how to estimate the SIFT points based on the
 * Normal gradients i.e. curvature than using the Intensity gradient
 * as usually used for SIFT keypoint estimation.
 */

int
main(int argc, char** argv)
{ 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
     std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd"); 
     std::string filename;
     
     if (!pcd_filename_indices.empty ())
  {
     filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, *cloud_xyz) == -1)
    {
      std::cout << "Was not able to open file \""<<filename<<"\".\n";
      return -1;
    }
  }
    else
  {
    std::cout << "\nNo *.pcd file given => closing.\n\n";
    return -1;
}


 std::cout << "points: " << cloud_xyz->points.size () <<std::endl;
  
  // Parameters for sift computation
  const float min_scale = 0.01f;
  const int n_octaves = 3;
  const int n_scales_per_octave = 4;
  const float min_contrast = 0.001f;
  
  // Estimate the normals of the cloud_xyz
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

  ne.setInputCloud(cloud_xyz);
  ne.setSearchMethod(tree_n);
  ne.setRadiusSearch(0.2);
  ne.compute(*cloud_normals);

  // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
  for(size_t i = 0; i<cloud_normals->points.size(); ++i)
  {
    cloud_normals->points[i].x = cloud_xyz->points[i].x;
    cloud_normals->points[i].y = cloud_xyz->points[i].y;
    cloud_normals->points[i].z = cloud_xyz->points[i].z;
  }

  // Estimate the sift interest points using normals values from xyz as the Intensity variants
  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud_normals);
  sift.compute(result);

  std::cout << "Number of SIFT points in the result are " << result.points.size () << std::endl;


  return 0;
  
}
