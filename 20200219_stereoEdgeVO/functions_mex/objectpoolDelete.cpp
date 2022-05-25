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
	if (nrhs != 2)
		mexErrMsgTxt("The number of inputs needs to be 2\n");

   // retrieve object pools' pointers
   double* ptr_prhs0 = mxGetPr(prhs[0]);
   double* ptr_prhs1 = mxGetPr(prhs[1]);
   intptr_t ptr0 = (intptr_t)(*ptr_prhs0);
   intptr_t ptr1 = (intptr_t)(*ptr_prhs1);

   ObjectPool<Node>* ptr_objpool_node = (ObjectPool<Node>*)ptr0;
   ObjectPool<Elem>* ptr_objpool_elem = (ObjectPool<Elem>*)ptr1;

   delete ptr_objpool_node;
   delete ptr_objpool_elem;
}