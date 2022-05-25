#include <iostream>
#include <vector>
#include <cmath>

#include "mex.h"
#include "matrix.h" //isNaN/isinf

#include "CommonStruct.h"
#include "mex_io.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray *prhs[])
{
   double* left_ptr;
   double* right_ptr;
   int n_elem;
   double invn_elem;

   left_ptr = mxGetPr(prhs[0]);
   if(left_ptr == NULL)
      mexErrMsgTxt("first input is not valid\n");
   right_ptr = mxGetPr(prhs[1]);
   if(right_ptr == NULL)
      mexErrMsgTxt("second input is not valid\n");

   
   n_elem = (int)mxGetM(prhs[0]);
   invn_elem = 1.0/(double)n_elem;

   double mean_l = 0;
   double mean_r = 0;
   double numer = 0;
   double denom_l = 0;
   double denom_r = 0;
   
   double cost_temp = -10;

   // calculate means
   for(int i = 0; i < n_elem; i++){
      mean_l += left_ptr[i];
      mean_r += right_ptr[i];
   }
   mean_l *= invn_elem;
   mean_r *= invn_elem;

   // calculate costs
   for(int i = 0; i < n_elem; i++){
      numer += (left_ptr[i] - mean_l)*(right_ptr[i] - mean_r);
      denom_l += (left_ptr[i] - mean_l)*(left_ptr[i] - mean_l);
      denom_r += (right_ptr[i] - mean_r)*(right_ptr[i] - mean_r);
   }
   cost_temp = numer/sqrt(denom_l*denom_r);

   // output
   plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
   double* plhs0_ptr = mxGetPr(plhs[0]);
   plhs0_ptr[0] = cost_temp;
}