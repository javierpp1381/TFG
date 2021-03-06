/* This example shows how to estimate the SIFT points based on the
 * Normal gradients i.e. curvature than using the Intensity gradient
 * as usually used for SIFT keypoint estimation.
 */

//------------------------------------------------------------------------------------------------
//------------------------------------------------LIBRARIES--------------------------------------
//------------------------------------------------------------------------------------------------
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

#include <fstream>

//------------------------------------------------------------------------------------------------
//-------------------------------------------GLOBAL VARIABLES-------------------------------------
//------------------------------------------------------------------------------------------------

clock_t begin,end;
double elapsed_sec;
float normal_estimation_time = 0.0f;
float sift_estimation_time = 0.0f;
// Parameters for sift computation
float min_scale = 0.01f;
int n_octaves = 3;
int n_scales_per_octave = 4;
float min_contrast = 0.001f;
int sift_points=0; 

//------------------------------------------------------------------------------------------------
//-------------------------------------------METHODS----------------------------------------------
//------------------------------------------------------------------------------------------------
void 
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-ms <float>	Minimum scale (default " << min_scale << ")\n"
            << "-no <int>	Number of octaves (default " << n_octaves << ")\n"
            << "-ns <int>	Number of scales per octave (default " << n_scales_per_octave << ")\n"
	    << "-mc <float>	Minimum contrast (default " << min_contrast << ")\n"
	    << "-h		Show help\n"
            << "\n\n";
}


//------------------------------------------------------------------------------------------
//-------------------------------------------MAIN-------------------------------------------	
//------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{


//---------------------------------------parse arguments----------------------------------

  if(argc == 1 || (pcl::console::find_argument (argc,argv,"-h") >= 0) ){
	printUsage (argv[0]);
	return 0;
  }	
  std::cout << std::endl << "---Sift points parameters---" << std::endl;
  
  pcl::console::parse (argc,argv, "-ms", min_scale);
  std::cout << "Setting minimum scale to: " << min_scale << std::endl;

  pcl::console::parse (argc,argv, "-no", n_octaves);
  std::cout << "Setting number of octaves to: " << n_octaves << std::endl;

  pcl::console::parse (argc,argv, "-ns", n_scales_per_octave);
  std::cout << "Setting number of scales per octave to: " << n_scales_per_octave << std::endl;

  pcl::console::parse (argc,argv, "-mc", min_contrast);
  std::cout << "Setting minimum contrast to: " << min_contrast << std::endl;

  std::cout << std::endl << std::endl;

//----------------------------------read input .pcd file-------------------------------------  
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

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd"); 
  std::string filename;
     
  std::cout << "Reading file..." << std::endl;

  if (!pcd_filename_indices.empty ())
  {
  	filename = argv[pcd_filename_indices[0]];
  	if (pcl::io::loadPCDFile (filename, *cloud_normals) == -1) 
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
  std::cout << "Number of points in "<< filename << ": "<< cloud_normals->points.size () <<std::endl; 
  std::cout << "Time needed for " << filename << " to load: " << elapsed_sec << " seconds"<< std::endl << std::endl; 

//------------------------Estimate the sift interest points using normals values from xyz as the Intensity variants------------
  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale>::Ptr result(new pcl::PointCloud<pcl::PointWithScale>);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud_normals);
 
  std::cout << "Estimating sift points in " << filename << "..." << std::endl;

  begin = clock();
  sift.compute(*result);
  end = clock();
  sift_estimation_time = double(end-begin)/CLOCKS_PER_SEC;
  std::cout << "Time needed for sift point extraction: " << sift_estimation_time << " seconds" << std::endl << std::endl;


//----------------------------save .pcd file with keypoints colored in green-----------------------------
  if(result->points.size()>0){
  
  	std::cout << "Number of SIFT points in " << filename << ": " << result->points.size () << std::endl;

	sift_points = result->points.size();

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
  else 
  {
  	std::cout << "No sift points found" << std::endl;
	sift_points = 0;
  }
 
  //-------------------------------store data in txt file----------------------------------
  std::fstream fs;
  fs.open("tests.txt", std::fstream::app);
  
  fs << "filename: " << filename << std::endl;
  
  fs << std::endl <<  "Normal estimation radius search: 0.02" << std::endl; 
  fs << "Minimum scale: " << min_scale << std::endl;
  fs << "Number of octaves: " << n_octaves << std::endl;
  fs << "Number of scales per octave: " << n_scales_per_octave << std::endl;
  fs << "Minimum contrast: " << min_contrast << std::endl;

  fs << "SIFT points estimation time (s): " << sift_estimation_time << std::endl;
  fs << "Number of SIFT points found: " << sift_points << std::endl;

  fs << "---------------------\n----------------------\n";
  fs.close();
  return 0;
  
}
