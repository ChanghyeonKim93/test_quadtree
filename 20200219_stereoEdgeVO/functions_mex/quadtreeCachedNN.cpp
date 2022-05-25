#include <iostream>
#include <vector>
#include <cmath>

#include "mex.h"
#include "matrix.h" //isNaN/isinf

#include "QuadTreeFastPooled.h"
#include "CommonStruct.h"
#include "mex_io.h"

void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[]) {
	// check number of arguments
	// 1: tree ptr, 2: points_query, 3: node_cached
	if (nrhs != 3)
		mexErrMsgTxt("This function requires 2 arguments\n");
	if (!mxIsNumeric(prhs[0]))
		mexErrMsgTxt("varargin{0} must be a valid quadtree root node pointer\n");
	if (!mxIsNumeric(prhs[1]))
		mexErrMsgTxt("varargin{1} must be a query set of points\n");

	// retrieve the tree pointer
	QuadTreeFastPooled* qt;
	retrieveTree(prhs[0], qt);
	// retrieve the query data
	std::vector<Point2<double>> points_query;
	std::vector<Node*> nodes_cached;
	int n_pts;
	int n_dims;
	retrieveData(prhs[1], points_query, n_pts, n_dims);
	retrieveNodes(prhs[2], nodes_cached);

	printf("query size: [%d x %d]\n", n_pts, n_dims);

	// retrieve the cached nodes.	
	// npoints x 1 indexes in output
	std::vector<int> index_matched;
	index_matched.reserve(n_pts);

	plhs[0] = mxCreateDoubleMatrix(n_pts, 1, mxREAL);
	plhs[1] = mxCreateDoubleMatrix(n_pts, 1, mxREAL);
	double* index_matched_output = mxGetPr(plhs[0]);
	double* nodes_matched_output = mxGetPr(plhs[1]);

	// execute the query FOR EVERY point storing the index
	// find matches
	qt->searchNNMultipleQueriesCached(points_query, index_matched, nodes_cached);

	for (int i = 0; i < n_pts; i++) {
		index_matched_output[i] = index_matched[i] + 1;
		nodes_matched_output[i] = (intptr_t)nodes_cached[i];
		//mexPrintf("addr:%d\n",cached_nodes_new_vec[i]);
	}
}