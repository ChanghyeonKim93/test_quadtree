
#include <iostream>
#include <vector>
#include <cmath>

#include "mex.h"
#include "matrix.h"//isNaN/isinf

#include "CommonStruct.h"
#include "mex_io.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray *prhs[])
{
    // 입력: (1) 이미지 , (2) 2 X N 짜리 보간 좌표들.
    // 넣어주는 보간 대상 이미지
    double* img_ptr;
    int n_cols, n_rows;

    // 보간 할 좌표들.
    double* pts_ptr;
    int n_pts, n_dims;

    if(!mxIsDouble(prhs[0]))
        mexErrMsgTxt("A unique double matrix of image should be passed.\n");
    if(nrhs == 2){
    }
    else mexErrMsgTxt("Wrong input style.\n");

    // 이미지 얻어오기
    retrieveImage(prhs[0], img_ptr, n_rows, n_cols); // edge 

    // 좌표 얻어오기.
    pts_ptr = mxGetPr(prhs[1]);
    if(pts_ptr == NULL)
        mexErrMsgTxt("vararg{2} must be a point array.\n");
    n_dims = (int)mxGetM(prhs[1]);
    n_pts  = (int)mxGetN(prhs[1]);
    if( n_dims != 2 )
        mexErrMsgTxt("The dimension of point array is not 2-d.\n");

    // 보간 된 결과가 담기는 double pointer.
    double* img_interped = new double[n_pts]; 
    // 제대로 복원된 점인지 아닌지를 나타내는 index들. (잘못되면 -1)
    double* idx_valid    = new double[n_pts];

    // 현재의 실수 좌표.
    double u_cur, v_cur;
    // 현재 좌표의 floor 좌표. (정수)
    int u_0, v_0;   
    int u_0n_rows, u_0n_rowsv_0;

    // 가중치들
    double ax, ay, axay;
    int is_valid = -1;
    // 나 자신과 이웃한 (동, 남, 동남) 픽셀들의 좌표
	double I1, I2, I3, I4;
    
    for(int i = 0; i < n_pts; i++) {
        u_cur = pts_ptr[2*i] - 1;   // col, double
        v_cur = pts_ptr[2*i+1] - 1; // row, double
        u_0   = (int)floor(u_cur);
        v_0   = (int)floor(v_cur);
        is_valid = 1;
        if(u_cur >= 0 && u_cur < n_cols-1)
            ax = u_cur - (double)u_0;
        else if(u_cur == n_cols-1){
            u_0 = (n_cols - 1) - 1;
            ax  = 0;
        }
        else if(u_cur > -1 && u_cur < 0){
            u_0 = 1;
            ax  = 1;
        }
        else
            is_valid = 0;
        
        if(v_cur >= 0 && v_cur < n_rows-1)
            ay = v_cur - (double)v_0;
        else if(v_cur == n_rows -1){
            v_0 = (n_rows - 1) - 1;
            ay = 0;
        }
        else if(v_cur > -1 && v_cur < 0){
            v_0 = 1;
            ay  = 1;
        }
        else
            is_valid = 0;

        if(is_valid == 1){
            u_0n_rows    = u_0*n_rows;
            u_0n_rowsv_0 = u_0n_rows + v_0;
//             img_interped[i] = (1-ax)*(1-ay)*img_ptr[u_0*n_rows+v_0] + ax*(1-ay)*img_ptr[(u_0+1)*n_rows+v_0] + (1-ax)*ay*img_ptr[u_0*n_rows+v_0+1] + ax*ay*img_ptr[(u_0+1)*n_rows+v_0+1];
            img_interped[i] = 0;
            // img_interped[i] += (1-ax)*(1-ay)*img_ptr[u_0*n_rows+v_0];
            // img_interped[i] += ax*(1-ay)*img_ptr[(u_0+1)*n_rows+v_0];
            // img_interped[i] += (1-ax)*ay*img_ptr[u_0*n_rows+v_0+1];
            // img_interped[i] += ax*ay*img_ptr[(u_0+1)*n_rows+v_0+1];
            // img_interped[i] = (1-ax)*(ay*(img_ptr[u_0*n_rows+v_0+1]-img_ptr[u_0*n_rows+v_0])+img_ptr[u_0*n_rows+v_0])+ax*(ay*(img_ptr[(u_0+1)*n_rows+v_0+1]-img_ptr[(u_0+1)*n_rows+v_0])+img_ptr[(u_0+1)*n_rows+v_0]);
            // I1 I2
            // I3 I4
            I1 = img_ptr[u_0n_rowsv_0]; // 자신
            I2 = img_ptr[u_0n_rowsv_0 + n_rows]; // 우
			I3 = img_ptr[u_0n_rowsv_0 + 1]; // 하
			I4 = img_ptr[u_0n_rowsv_0 + n_rows + 1]; // 우하
            
            img_interped[i] += axay*(I1 - I2 - I3 + I4);
			img_interped[i] += ax*(-I1 + I2);
			img_interped[i] += ay*(-I1 + I3);
			img_interped[i] += I1;
            idx_valid[i]    = 1;
        }
        else{
            img_interped[i] = -2;
            idx_valid[i]    = 0;
        }
    }
    
    // 출력. 보간된 이미지 & validity index들.
    plhs[0]=mxCreateDoubleMatrix(n_pts,1,mxREAL);
    plhs[1]=mxCreateDoubleMatrix(n_pts,1,mxREAL);

    double* plhs0_ptr = mxGetPr(plhs[0]);
    double* plhs1_ptr = mxGetPr(plhs[1]);
    for(int i = 0; i < n_pts; i++) {
        plhs0_ptr[i] = img_interped[i];
        plhs1_ptr[i] = idx_valid[i];
    }
    
    delete[] img_interped;
    delete[] idx_valid;
}