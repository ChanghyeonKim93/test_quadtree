#include <iostream>
#include <vector>
#include <cmath>

#include "mex.h"
#include "matrix.h" //isNaN/isinf

#include "QuadTreeFastPooled.h"
#include "CommonStruct.h"
#include "mex_io.h"

void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[]){   
	// check input
   if( nrhs!=1 || !mxIsNumeric(prhs[0]) )
      mexErrMsgTxt("varargin{1} must be a valid quadtree pointer\n");

   // retrieve the tree pointer
   double* pointer0 = mxGetPr(prhs[0]);
   intptr_t pointer1 = (intptr_t)pointer0[0];
   // convert it to "QuadTree Node*"
   QuadTreeFastPooled* qt = (QuadTreeFastPooled*)pointer1;
   delete qt;
}
