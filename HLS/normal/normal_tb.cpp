#include "normal.h"

int main(){

	float cloud[POINTS][3]={0};
	int indices[INDICES]={0};
	float covariance_matrix[3][3]={0};
	float centroid[4]={0};
	int i=0;

	//-----------------------------------------------------cosas--------------------------------------------
	printf("\n covariance matrix before\n");

	for(i=0;i<3;i++){
			for(int j=0;j<3;j++){
				printf(" %f ", covariance_matrix[i][j]);
			}
			printf("\n");
	}


	//-------------------------------------------lectura de nube de puntos---------------------------------------------

	printf("\npoint cloud\n");
	FILE *file = fopen("/home/ubuntu/pcl/sift_keypoints_bueno/build/neighbors/cloud.txt", "r");

	i=0;
	while (!feof(file)){
		fscanf(file,"%f %f %f",&cloud[i][0], &cloud[i][1], &cloud[i][2]);

		printf("%f %f %f\n",cloud[i][0], cloud[i][1], cloud[i][2]);
		i++;
	}

	fclose(file);

	file = NULL;

	//------------------------------------------------lectura de indices----------------------------------------------------------
	printf("\nindices\n");

	file = fopen("/home/ubuntu/pcl/sift_keypoints_bueno/build/neighbors/indices.txt", "r");

	i=0;
	while(i<INDICES){
			fscanf(file,"%d",&indices[i]);

			printf("%d ",indices[i]);
			i++;
		}

	fclose(file);

	//----------------------------------------------------estima matriz covarianzas---------------------------------------------
	compute(covariance_matrix,centroid,cloud,indices);

	//----------------------------------------------------------compruebo resultados--------------------------------------------------
	printf("\n\ncovariance matrix after\n");

	for(i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf(" %f ", covariance_matrix[i][j]);
		}
		printf("\n");
	}


	return 0;
}
