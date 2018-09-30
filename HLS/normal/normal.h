#include <stdio.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <stdlib.h>
#include <vector>

#define POINTS 13704
#define INDICES 223

int compute(volatile float covariance_matrix[3][3],volatile float centroid[4],volatile float cloud[10][3],volatile int indices[5]);
