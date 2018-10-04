#include "normal.h"

int main(){

	float cloud[MAXPOINTS][3]={0};
	int indices[MAXINDICES]={0};
	float covariance_matrix[3][3]={0};
	float centroid[4]={0};
	unsigned int i=0;
	int num_points=0;
	int num_indices=0;
	//-----------------------------------------------------------------------------------------------------------------
	//--------------------------------------------muestro matriz covarianzas y centroide inicializados--------------------------------------------
	//-----------------------------------------------------------------------------------------------------------------

	printf("\n covariance matrix before\n");

	for(i=0;i<3;i++){
			for(int j=0;j<3;j++){
				printf(" %f ", covariance_matrix[i][j]);
			}
			printf("\n");
	}

	printf("\n centroid before\n");

	printf("%f %f %f %f\n", centroid[0], centroid[1], centroid[2], centroid[3]);


	//-----------------------------------------------------------------------------------------------------------------
	//-------------------------------------------lectura de nube de puntos---------------------------------------------
	//-----------------------------------------------------------------------------------------------------------------


	//printf("\npoint cloud\n");
	FILE *file = fopen("/home/embedded/pcl/sift_keypoints/build/results/cloud.txt", "r");

	i=0;
	fscanf(file,"%d",&num_points);
	while (i<num_points){
		fscanf(file,"%f %f %f",&cloud[i][0], &cloud[i][1], &cloud[i][2]);

		//printf("%f %f %f\n",cloud[i][0], cloud[i][1], cloud[i][2]);
		i++;
	}

	fclose(file);

	file = NULL;

	//-----------------------------------------------------------------------------------------------------------------
	//------------------------------------------------lectura de indices----------------------------------------------------------
	//-----------------------------------------------------------------------------------------------------------------

	printf("\nindices\n");

	file = fopen("/home/embedded/pcl/sift_keypoints/build/results/indices.txt", "r");

	fscanf(file,"%d",&num_indices);

	i=0;
	while(i<num_indices){
			fscanf(file,"%d",&indices[i]);

			printf("%d ",indices[i]);
			i++;
	}

	fclose(file);
	file = NULL;

	//-----------------------------------------------------------------------------------------------------------------
	//----------------------------------------------------estima matriz covarianzas---------------------------------------------
	//-----------------------------------------------------------------------------------------------------------------

	compute(covariance_matrix,centroid,cloud,indices,num_points,num_indices);

	//-----------------------------------------------------------------------------------------------------------------
	//----------------------------------------------------------resultados de HLS--------------------------------------------------
	//-----------------------------------------------------------------------------------------------------------------

	printf("\n\ncovariance matrix from HLS\n");

	for(i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf(" %f ", covariance_matrix[i][j]);
		}
		printf("\n");
	}

	printf("\n centroid from HLS\n");

	printf("%f %f %f %f\n", centroid[0], centroid[1], centroid[2], centroid[3]);

	//-----------------------------------------------------------------------------------------------------------------
	//----------------------------------------------------------resultados de PCL--------------------------------------------------
	//-----------------------------------------------------------------------------------------------------------------



	printf("\n\ncovariance matrix from PCL\n");

	file = fopen("/home/embedded/pcl/sift_keypoints/build/results/covariance_matrix_PCL.txt", "r");

	float aux[4]={0};

	while (!feof(file)){
		fscanf(file,"%f %f %f",&aux[0], &aux[1], &aux[2]);

		printf("%f %f %f\n",aux[0], aux[1], aux[2]);

	}
	fclose(file);
	file = NULL;

	printf("\ncentroid from PCL\n");

	file = fopen("/home/embedded/pcl/sift_keypoints/build/results/centroid_PCL.txt", "r");


	while (!feof(file)){
		fscanf(file,"%f %f %f %f",&aux[0], &aux[1], &aux[2],&aux[3]);

		printf("%f %f %f %f\n",aux[0], aux[1], aux[2], aux[3]);

	}
	fclose(file);
	file = NULL;

	//-----------------------------------------------------------------------------------------------------------------
	//----------------------------------------------------------guardo resultados--------------------------------------------------
	//-----------------------------------------------------------------------------------------------------------------

	/*file = fopen("/home/embedded/pcl/sift_keypoints/build/results/centroid_HLS.txt", "w");

	fprintf(file,"%f %f %f %f\n",centroid[0],centroid[1],centroid[2],centroid[3]);

	fclose(file);
	file = NULL;
/*
	file = fopen("/home/embedded/pcl/sift_keypoints/build/results/covariance_matrix_HLS.txt", "w");

	fprintf(file,"%f %f %f\n",covariance_matrix[0][0],covariance_matrix[0][1],covariance_matrix[0][2]);
	fprintf(file,"%f %f %f\n",covariance_matrix[1][0],covariance_matrix[1][1],covariance_matrix[1][2]);
	fprintf(file,"%f %f %f",covariance_matrix[2][0],covariance_matrix[2][1],covariance_matrix[2][2]);

	fclose(file);
	file = NULL;
*/
	return 0;
}
