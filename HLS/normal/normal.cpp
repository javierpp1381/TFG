#include "normal.h"

int compute (volatile float covariance_matrix[3][3],volatile float centroid[4],volatile float cloud[MAXPOINTS][3],volatile int indices[MAXINDICES], int num_points, int num_indices){

	/*#pragma HLS INTERFACE axis port=covariance_matrix
	#pragma HLS INTERFACE axis port=centroid

	#pragma HLS INTERFACE axis port=cloud
	#pragma HLS INTERFACE axis port=indices*/


	float accu[9]={0};

	for(int i = 0;i<num_indices;i++){
		accu[0] += cloud[indices[i]][0] * cloud[indices[i]][0];
		accu[1] += cloud[indices[i]][0] * cloud[indices[i]][1];
		accu[2] += cloud[indices[i]][0] * cloud[indices[i]][2];
		accu[3] += cloud[indices[i]][1] * cloud[indices[i]][1];
		accu[4] += cloud[indices[i]][1] * cloud[indices[i]][2];
		accu[5] += cloud[indices[i]][2] * cloud[indices[i]][2];
		accu[6] += cloud[indices[i]][0];
		accu[7] += cloud[indices[i]][1];
		accu[8] += cloud[indices[i]][2];
	}

	for(int i = 0;i<9;i++){
		accu[i]/=num_indices;
	}
	centroid[0] = accu[6];
	centroid[1] = accu[7];
	centroid[2] = accu[8];
	centroid[3] = 1;

	covariance_matrix[0][0] = accu [0] - accu [6] * accu [6];
	covariance_matrix[0][1] = accu [1] - accu [6] * accu [7];
	covariance_matrix[0][2] = accu [2] - accu [6] * accu [8];
	covariance_matrix[1][1] = accu [3] - accu [7] * accu [7];
	covariance_matrix[1][2] = accu [4] - accu [7] * accu [8];
	covariance_matrix[2][2] = accu [5] - accu [8] * accu [8];

	covariance_matrix[1][0] = accu [1] - accu [6] * accu [7];
	covariance_matrix[2][0] = accu [2] - accu [6] * accu [8];
	covariance_matrix[2][1] = accu [4] - accu [7] * accu [8];

	/*covariance_matrix[1][0] = covariance_matrix[0][1];
	covariance_matrix[2][0] = covariance_matrix[0][2];
	covariance_matrix[2][1] = covariance_matrix[1][2];*/

return 0;


}
