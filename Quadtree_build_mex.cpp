#include "mex.h"
#include "matrix.h" //isNaN/isinf
#include <vector>
#include <iostream>
#include <cmath>
#include "Quadtree.h"

// matlab entry point
void retrieve_data( const mxArray* matptr, std::vector<Point>& dataV, int& npoints, int& ndims){
    // retrieve pointer from the MX form
    double* data = mxGetPr(matptr);
    // check that I actually received something
    if( data == NULL )
        mexErrMsgTxt("vararg{1} must be a [npoints x ndims] matrix of data\n");
    
    // retrieve amount of points
    npoints = mxGetM(matptr);
    ndims   = mxGetN(matptr);
    if(ndims != 2)
        mexErrMsgTxt("ndims must be 2.\n");

    // Make sure !nan & !inf
    for( int i=0; i < npoints*ndims; i++ ){
        if( mxIsNaN( data[i] ) ) mexErrMsgTxt("input data contains NAN values.");
        if( mxIsInf( data[i] ) ) mexErrMsgTxt("input data contains INF values!");
    }
    
    // FILL THE DATA STRUCTURES
	for( int i=0; i<npoints; i++ ){
        dataV.push_back(Point(data[i],data[i+npoints],i));
    }
}
void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[]){   
	// check input
	if( !mxIsNumeric(prhs[0]) )
		mexErrMsgTxt("A unique [npoints x ndims] matrix of points should be passed.\n");
	if( nrhs != 5)
        mexErrMsgTxt("The number of inputs needs to be 5.\n");

    // retrieve the data
    std::vector<Point> input_data;
    int npoints;
    int ndims;
    retrieve_data( prhs[0], input_data, npoints, ndims );
    
    printf("npoints %d ndims %d\n", npoints, ndims);
    
    // fill the Quadtree
    double* n_rows    = mxGetPr(prhs[1]);
    double* n_cols    = mxGetPr(prhs[2]);
    double* max_depth = mxGetPr(prhs[3]);
    double* eps       = mxGetPr(prhs[4]);

    QuadTree* tree = new QuadTree(input_data, *n_rows, *n_cols, (int)(*max_depth), *eps);
 
  /*
    start = clock();
    std::vector<int> indexVec;
    indexVec.reserve(numOfPoints);
    bkdTree2->kdtree_nearest_neighbor_approximate(points_q_vec, indexVec);
    finish = clock();
    */

    
	// DEBUG
 	//mexPrintf("npoint %d dimensions %d\n", (int)input_data.size(), (int)input_data[0].size());
	
	// DEBUG
	//for (unsigned int i=0; i < input_data.size(); i++){
	//	for (unsigned int j=0; j < input_data[i].size(); j++)
	//		mexPrintf("%.2f ", input_data[i][j] );
	//	mexPrintf("\n");
	//}

    // return the program a pointer to the created tree
    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
    double* pointer_to_tree = mxGetPr(plhs[0]);
    pointer_to_tree[0] = (intptr_t) (tree);
} 