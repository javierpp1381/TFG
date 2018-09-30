/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_NORMAL_3D_H_
#define PCL_FEATURES_IMPL_NORMAL_3D_H_

#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::NormalEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  pcl::PointCloud<pcl::PointXYZ> neighbors;
  neighbors.height=1;

  output.is_dense = true;

  //std::string id_neighbors_str("0");
  //int id_neighbors_int=0;


  const char *file_name;

  FILE *file;
 
  //save point cloud in txt
  file_name = "/home/ubuntu/pcl/sift_keypoints_bueno/build/neighbors/cloud.txt";
  file=fopen(file_name,"w");
  size_t i=0; 
  for(i=0; i<surface_->points.size()-1;i++){
  	fprintf(file,"%f %f %f \n",surface_->points[i].x,surface_->points[i].y,surface_->points[i].z);
  }
  fprintf(file,"%f %f %f ",surface_->points[i].x,surface_->points[i].y,surface_->points[i].z);

  fclose(file);
  file=NULL;
  
  // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
  if (input_->is_dense)
  {
    // Iterating over the entire index vector
    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0 ||
          !computePointNormal (*surface_, nn_indices, output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2], output.points[idx].curvature))
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }


      //save last neighbors indices in a txt
	/*neighbors.width = nn_indices.size();
	neighbors.is_dense=false;
	neighbors.resize(neighbors.height*neighbors.width);
	*/
	
//	file_name = ("/home/ubuntu/pcl/sift_keypoints_bueno/build/neighbors/fichero_prueba"+id_neighbors_str+".txt").c_str();
	file_name ="/home/ubuntu/pcl/sift_keypoints_bueno/build/neighbors/indices.txt";

	file = fopen(file_name,"w");

	for(size_t i=0; i<nn_indices.size();i++){
		//neighbors.points[i].x = surface_->points[nn_indices[i]].x;
		//neighbors.points[i].y = surface_->points[nn_indices[i]].y;
		//neighbors.points[i].z = surface_->points[nn_indices[i]].z;
		//fprintf(file, "%f %f %f \n",surface_->points[nn_indices[i]].x,surface_->points[nn_indices[i]].y,surface_->points[nn_indices[i]].z);	
		fprintf(file, "%d\n",nn_indices[i]);	

		
		//std::cerr << "    " << surface_->points[i].x << " " << surface_->points[i].y << " " << surface_->points[i].z << std::endl;
	}

	//fprintf(file,"\n*\n");

  	fclose(file);
	
	file = NULL;
	
	//pcl::io::savePCDFileASCII ("/home/ubuntu/pcl/sift_keypoints_bueno/build/neighbors/neighbors_cloud_number_"+id_neighbors_str+".pcd", neighbors);
        //id_neighbors_int=std::stoi(id_neighbors_str);
	//id_neighbors_int++;
	//id_neighbors_str=std::to_string(id_neighbors_int);


	//save last neighbors covariance matrix in a txt
	file_name ="/home/ubuntu/pcl/sift_keypoints_bueno/build/neighbors/covariance_matrix.txt";

	file=fopen(file_name,"w");
	
	fprintf(file,"%f %f %f\n",covariance_matrix_(0,0),covariance_matrix_(0,1),covariance_matrix_(0,2));
	fprintf(file,"%f %f %f\n",covariance_matrix_(1,0),covariance_matrix_(1,1),covariance_matrix_(1,2));
	fprintf(file,"%f %f %f",covariance_matrix_(2,0),covariance_matrix_(2,1),covariance_matrix_(2,2));

	fclose(file);
	file = NULL;

	flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);

    }
  }
  else
  {
    // Iterating over the entire index vector
    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (!isFinite ((*input_)[(*indices_)[idx]]) ||
          this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0 ||
          !computePointNormal (*surface_, nn_indices, output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2], output.points[idx].curvature))
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);

    }
  }
}

#define PCL_INSTANTIATE_NormalEstimation(T,NT) template class PCL_EXPORTS pcl::NormalEstimation<T,NT>;

#endif    // PCL_FEATURES_IMPL_NORMAL_3D_H_ 
