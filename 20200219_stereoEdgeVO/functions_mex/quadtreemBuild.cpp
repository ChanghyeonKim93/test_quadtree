#include <iostream>
#include <vector>
#include <cmath>

#include "mex.h"
#include "matrix.h" //isNaN/isinf

#include "QuadTreeFastMultiplePooled.h"
#include "mex_io.h"

void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[]) {
	// check input
	if (!mxIsNumeric(prhs[0]))
		mexErrMsgTxt("A unique [npoints x ndims] matrix of points should be passed.\n");
	if (nrhs != 10)
		mexErrMsgTxt("The number of inputs needs to be 10\n");

	// 1: points, 2: dir1, 3: dir2, 4: n_rows, 5: n_cols, 6: max_depth, 7: eps, 8: dist_thres
	// 9: objpool_node, 10: objpool_elem

	// retrieve the data
	std::vector<Point2<double>> points;
   std::vector<int> dirs1;
   std::vector<int> dirs2;
	int n_pts;
	int n_dims;
   int dum1, dum2;
	retrieveData(prhs[0], points, n_pts, n_dims);
   retrieveDirections(prhs[1], dirs1);
   retrieveDirections(prhs[2], dirs2);

	// fill the Quadtree
	double* n_rows     = mxGetPr(prhs[3]);
	double* n_cols     = mxGetPr(prhs[4]);
	double* max_depth  = mxGetPr(prhs[5]);
	double* eps        = mxGetPr(prhs[6]);
	double* dist_thres = mxGetPr(prhs[7]);
	
	double* ptr_prhs0  = mxGetPr(prhs[8]);
   double* ptr_prhs1  = mxGetPr(prhs[9]);
   intptr_t ptr0 = (intptr_t)(*ptr_prhs0);
   intptr_t ptr1 = (intptr_t)(*ptr_prhs1);
	
	ObjectPool<Node>* objpool_node = (ObjectPool<Node>*)(ptr0);
	ObjectPool<Elem>* objpool_elem = (ObjectPool<Elem>*)(ptr1);

	QuadTreeFastMultiplePooled* qt_mult = 
   new QuadTreeFastMultiplePooled(points, dirs1, dirs2, *n_rows, *n_cols, (int)(*max_depth), *eps, *dist_thres, objpool_node, objpool_elem);

	objpool_node->showRemainedMemory();
	objpool_elem->showRemainedMemory();
   // 하나의 scope 안에서는 잘 동작 하는데 ....
   // delete qt_mult;

	// return the program a pointer to the created tree
	plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
	
	double* ptr_tree = mxGetPr(plhs[0]);
	ptr_tree[0] = (intptr_t)(qt_mult);
}