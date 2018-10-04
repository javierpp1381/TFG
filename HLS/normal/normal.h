#include <stdio.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <stdlib.h>
#include <vector>

#define MAXPOINTS 50000
#define MAXINDICES 400

int compute(volatile float covariance_matrix[MAXPOINTS][3],volatile float centroid[4],volatile float cloud[MAXPOINTS][3],volatile int indices[MAXINDICES],int num_points, int num_indices);
