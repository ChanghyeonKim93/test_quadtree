#include <iostream>
#include <vector>
#include <cmath>

#include "mex.h"
#include "matrix.h" //isNaN/isinf

#include "CommonStruct.h"
#include "ObjectPool.h"
#include "mex_io.h"

void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[]) {
	// check input
	if (!mxIsNumeric(prhs[0]))
		mexErrMsgTxt("A maximum object size is needed.\n");
	if (nrhs != 1)
		mexErrMsgTxt("The number of inputs needs to be 1\n");

	// 1: MAX_NUM_OBJ, 
   // Node 와 Elem에 대한 object pool을 두개 다 만든다.
   double* max_num_obj_input = mxGetPr(prhs[0]);
	int MAX_NUM_OBJ = (int)(*max_num_obj_input);
   printf("max num obj: %d\n", MAX_NUM_OBJ);
   ObjectPool<Node>* objpool_node = new ObjectPool<Node>(MAX_NUM_OBJ);
   ObjectPool<Elem>* objpool_elem = new ObjectPool<Elem>(MAX_NUM_OBJ);

	printf("memory allocation- success.\n");
	// return the program a pointer to the created tree
	plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
   plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);

	double* ptr_objpool_node = mxGetPr(plhs[0]);
   double* ptr_objpool_elem = mxGetPr(plhs[1]);

	*ptr_objpool_node = (intptr_t)(objpool_node);
   *ptr_objpool_elem = (intptr_t)(objpool_elem);
}