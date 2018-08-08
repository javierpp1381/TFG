
//-----------------------------------------------
//-------------------LIBRARIES-------------------
//-----------------------------------------------
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
    
int user_data;
    
//-----------------------------------------------
//-------------------METHODS---------------------
//-----------------------------------------------
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 0.5);
    pcl::PointXYZ o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.01, "sphere", 0);
    
}
    
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    user_data++;
}


//-----------------------------------------------
//-------------------MAIN------------------------
//-----------------------------------------------
    
int 
main (int argc, char** argv)
{   

    //------------------------------read input cloud---------------------------------
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd"); 
    std::string filename;
     
    if (!pcd_filename_indices.empty ())
    {
    	filename = argv[pcd_filename_indices[0]];
    	if (pcl::io::loadPCDFile (filename, *cloud) == -1)
    	{
      		cerr << "Was not able to open file \""<<filename<<"\".\n";
      		return -1;
    	}
    }
    else
    {
    	cout << "\nNo *.pcd file given => closing.\n\n";
    	return -1;
    }
    
    cout << "\nNumber of points in "<< filename << ": " << cloud->points.size() << "\n";
    
    //--------------------visualize pointcloud----------------------------------------
    
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    viewer.showCloud(cloud);
    
    
    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread (viewerPsycho);
    
    
    while (!viewer.wasStopped ())
    {
    
    }
    return 0;
}
