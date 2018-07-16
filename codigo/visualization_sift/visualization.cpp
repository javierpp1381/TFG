#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
    
int user_data;
    
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sift_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 0.5);
    pcl::PointXYZ o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.01, "central sphere", 0);
    for(size_t i=0;i<sift_cloud->points.size();i++){
    	viewer.addSphere(sift_cloud->points[i],0.02f,50,255,50,"sphere"+i);	
    }
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
    
int 
main (int argc, char** argv)
{   

   /* 
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
   */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
   
    pcl::io::loadPCDFile(argv[1],*cloud);
    pcl::io::loadPCDFile(argv[2],*sift_cloud);


    cout << "\nNumber of points in input cloud: " << cloud->points.size() << "\n";
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud,"cloud");
    viewer.showCloud(sift_cloud,"sift_cloud");
   
     
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
    
    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
    }
    return 0;
}
