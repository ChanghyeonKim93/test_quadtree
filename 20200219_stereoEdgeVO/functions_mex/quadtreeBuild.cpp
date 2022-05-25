#include <iostream>
#include <vector>
#include <cmath>

#include "mex.h"
#include "matrix.h" //isNaN/isinf

#include "QuadTreeFastPooled.h"
#include "CommonStruct.h"
#include "mex_io.h"

void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[]) {
	// check input
	if (!mxIsNumeric(prhs[0]))
		mexErrMsgTxt("A unique [npoints x ndims] matrix of points should be passed.\n");
	if (nrhs != 8)
		mexErrMsgTxt("The number of inputs needs to be 8\n");

	// 1: points, 2: n_rows, 3: n_cols, 4: max_depth, 5: eps, 6: dist_thres
	// 7: objpool_node, 8: objpool_elem

	// retrieve the data
	std::vector<Point2<double>> points;
	int n_pts;
	int n_dims;
	retrieveData(prhs[0], points, n_pts, n_dims);

	// fill the Quadtree
	double* n_rows     = mxGetPr(prhs[1]);
	double* n_cols     = mxGetPr(prhs[2]);
	double* max_depth  = mxGetPr(prhs[3]);
	double* eps        = mxGetPr(prhs[4]);
	double* dist_thres = mxGetPr(prhs[5]);

	double* ptr_prhs0  = mxGetPr(prhs[6]);
   double* ptr_prhs1  = mxGetPr(prhs[7]);
   intptr_t ptr0 = (intptr_t)(*ptr_prhs0);
   intptr_t ptr1 = (intptr_t)(*ptr_prhs1);
	
	ObjectPool<Node>* objpool_node = (ObjectPool<Node>*)(ptr0);
	ObjectPool<Elem>* objpool_elem = (ObjectPool<Elem>*)(ptr1);
	
	//mexPrintf("dist thres : %lf\n",*dist_thres);
	mexPrintf("n_pts: %d / ndims: %d\n", n_pts, n_dims);
	QuadTreeFastPooled* qt = new QuadTreeFastPooled(points, *n_rows, *n_cols, (int)(*max_depth), *eps, *dist_thres, objpool_node, objpool_elem);

	// return the program a pointer to the created tree
	plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
	
	double* ptr_tree = mxGetPr(plhs[0]);
	ptr_tree[0] = (intptr_t)(qt);
}