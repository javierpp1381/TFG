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
//includes de javi
#include <iostream>
#include <ctime>
///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::NormalEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  clock_t begin,end;
  double elapsed_sec_neighbors, elapsed_sec_computePointNormal, elapsed_sec_flipNormal;
elapsed_sec_neighbors = 0;
 elapsed_sec_computePointNormal = 0; 
elapsed_sec_flipNormal = 0;

 int neighbors;
  bool pointNormal;

  double time_covariance = 0;
  double time_solvePlaneParameters = 0;

  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  output.is_dense = true;
  // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
  if (input_->is_dense)
  {
    // Iterating over the entire index vector
    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {

	begin = clock();
	 neighbors = this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);
	end = clock();
	elapsed_sec_neighbors += double (end-begin)/CLOCKS_PER_SEC;

	begin = clock();
	 pointNormal = computePointNormal (*surface_, nn_indices, output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2], output.points[idx].curvature, time_covariance, time_solvePlaneParameters); 
	end = clock();
	elapsed_sec_computePointNormal += double (end-begin)/CLOCKS_PER_SEC;

	if(neighbors == 0  || !pointNormal) 
      	{
        	output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

	        output.is_dense = false;
	        continue;
	}

	begin = clock();
	flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_, output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);
	end = clock();
	elapsed_sec_flipNormal += double (end-begin)/CLOCKS_PER_SEC;
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

  std::cerr << "########## TIME MEASUREMENT IN: pcl::normalEstimation computeFeature ##########" << std::endl;

 std::cerr << std::endl << "TOTAL TIME FOR SEARCHING NEIGHBORS: " << elapsed_sec_neighbors << " seconds" << std::endl;

  std::cerr << "########## TIME MEASUREMENT IN: pcl::normalEstimation computePointNormal ##########" << std::endl;

  std::cerr << "TOTAL TIME FOR CALCULATING COVARIANCE MATRIX: "<< time_covariance << " seconds" << std::endl;

  std::cerr << "TOTAL TIME FOR CALCULATING EIGENVALUES AND EIGENVECTORS: " << time_solvePlaneParameters << " seconds" << std::endl;
  
  std::cerr << "########## END OF TIME MEASUREMENT IN: pcl::normalEstimation computePointNormal ##########" << std::endl;

 std::cerr << std::endl << "TOTAL TIME FOR COMPUTE POINT NORMAL: " << elapsed_sec_computePointNormal << " seconds" << std::endl;


 std::cerr << std::endl << "TOTAL TIME FOR FLIP NORAL TOWARDS VIEWPOINT: " << elapsed_sec_flipNormal << " seconds" << std::endl << std::endl;

 std::cerr << "########## END OF TIME MEASUREMENT IN: pcl::normalEstiamtion computeFeature ##########" << std::endl;


}

#define PCL_INSTANTIATE_NormalEstimation(T,NT) template class PCL_EXPORTS pcl::NormalEstimation<T,NT>;

#endif    // PCL_FEATURES_IMPL_NORMAL_3D_H_ 
