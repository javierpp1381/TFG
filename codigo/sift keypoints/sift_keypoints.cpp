// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

#include <pcl/point_types.h>

#include <pcl/common/io.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>

//#include <pcl/console/print.cpp> //for cross compiling

#include <pcl/impl/pcl_base.hpp>///////////////

#include <pcl/search/pcl_search.h>
#include <pcl/search/impl/search.hpp>///
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/filters/impl/voxel_grid.hpp>

#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <ctime>

#include <boost/thread/thread.hpp>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/console/parse.h>
/* This example shows how to estimate the SIFT points based on the
 * Normal gradients i.e. curvature than using the Intensity gradient
 * as usually used for SIFT keypoint estimation.
 */

int
main(int argc, char** argv)
{

//time measurement variables
  clock_t begin,end;
  double elapsed_sec;
  
  
//read input .pcd file  
  begin = clock();

/*
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
 
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_xyz) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return -1;
  }
  */

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
  
  end = clock();
  elapsed_sec = double(end-begin)/CLOCKS_PER_SEC;
  std::cout << "Time needed for " << filename << " to load: " << elapsed_sec << " seconds"<< std::endl; 
  std::cout << "Number of points in "<< filename << ": "<< cloud_xyz->points.size () <<std::endl; 
 
// Estimate the normals of the cloud_xyz
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

  ne.setInputCloud(cloud_xyz);
  ne.setSearchMethod(tree_n);
  ne.setRadiusSearch(0.02);
 
  begin = clock();
  ne.compute(*cloud_normals);
  end = clock();
 
  elapsed_sec = double(end-begin)/CLOCKS_PER_SEC;
  std::cout << "Time needed for normal estimation in " << filename << ": " << elapsed_sec << std::endl;
 
// Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
  begin = clock();

  for(size_t i = 0; i<cloud_normals->points.size(); ++i)
  {
  	cloud_normals->points[i].x = cloud_xyz->points[i].x;
  	cloud_normals->points[i].y = cloud_xyz->points[i].y;
  	cloud_normals->points[i].z = cloud_xyz->points[i].z;
  }

  end = clock();
  elapsed_sec = double(end-begin)/CLOCKS_PER_SEC;
  std::cout << "time needed for copying the pointcloud: " << elapsed_sec << std::endl;

// Parameters for sift computation
  const float min_scale = 0.01f;//0.01f
  const int n_octaves = 3;//3
  const int n_scales_per_octave = 4;//4
  const float min_contrast = 0.001f;//0.001f
 
// Estimate the sift interest points using normals values from xyz as the Intensity variants
  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale>::Ptr result(new pcl::PointCloud<pcl::PointWithScale>);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud_normals);
 
  //pcl:PointCloud<int> keypoint_indices;
  //sift.compute(keypoint_indices);

    //for (size_t i=0; i<keypoint_indices.points.size (); ++i){
      //std::cout << " " << keypoint_indices.points[i] << " ";
    //}

  begin = clock();
  sift.compute(*result);
  end = clock();
  elapsed_sec = double(end-begin)/CLOCKS_PER_SEC;
  std::cout << "Time needed for sift point extraction: " << elapsed_sec << "\n";


//save .pcd file with keypoints colored in green
  if(result->points.size()>0){
  
  	std::cout << "Number of SIFT points in " << filename << ": " << result->points.size () << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
  
	keypoints->width = result->width;
	keypoints->height = result->height;
	keypoints->points.resize(keypoints->width * keypoints->height);

   	for (size_t i = 0; i < result->points.size (); ++i)
  	{
    		keypoints->points[i].x = result->points[i].x;
    		keypoints->points[i].y = result->points[i].y;
    		keypoints->points[i].z = result->points[i].z;

  		keypoints->points[i].r=50;
  		keypoints->points[i].g=255;
  		keypoints->points[i].b=50;
  		keypoints->points[i].a=255;
  	}
  	pcl::io::savePCDFileASCII ("sift_keypoints.pcd", *keypoints);
  }
  else {
  	std::cout << "No sift points found" << std::endl;
  }
  
  return 0;
  
}
