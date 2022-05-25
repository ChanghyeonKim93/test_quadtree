#ifndef _MEX_IO_H_
#define _MEX_IO_H_

#include <iostream>
#include <vector>

#include "CommonStruct.h"
#include "QuadTreeFastPooled.h"
#include "QuadTreeFastMultiplePooled.h"

#include "mex.h"
#include "matrix.h" //isNaN/isinf

void retrieveData(const mxArray *ptr_mat_, std::vector<Point2<double>> &points_, int &n_pts_, int &n_dims_)
{
    // retrieve pointer from the MX form
    double *data = mxGetPr(ptr_mat_);
    if(data == nullptr)
        mexErrMsgTxt("vararg{1} must be a [npoints x ndims] matrix of data.\n");
    
    n_pts_  = mxGetM(ptr_mat_);
    n_dims_ = mxGetN(ptr_mat_);
    if(n_dims_ != 2)
		mexErrMsgTxt("n_dims_ must be 2.\n");
    
    // fill the data
    points_.resize(0);
    for(int i = 0; i < n_pts_; i++){
        points_.push_back(Point2<double>(data[i], data[i+n_pts_]));
    }
};

void retrieveDirections(const mxArray *ptr_mat_, std::vector<int> &dirs_)
{
    // retrieve pointer from the MX form
    double *data = mxGetPr(ptr_mat_);
    if(data == nullptr)
        mexErrMsgTxt("vararg{1} must be a [npoints x ndims] matrix of data.\n");
    int n_pts_  = mxGetM(ptr_mat_);
    // fill the data
    dirs_.resize(0);
    for(int i = 0; i < n_pts_; i++){
        dirs_.push_back((int)data[i]);
    }
};

void retrieveNodes(const mxArray *ptr_mat_, std::vector<Node*> &nodes_)
{
    // retrieve pointer from the MX form
    double *data = mxGetPr(ptr_mat_);
    if(data == nullptr)
        mexErrMsgTxt("vararg{1} must be a [npoints x ndims] matrix of data.\n");
    int n_pts_  = mxGetM(ptr_mat_);
    // fill the data
    nodes_.resize(0);
    intptr_t ptr_temp;
    for(int i = 0; i < n_pts_; i++){
        ptr_temp = (intptr_t)data[i];
        nodes_.push_back((Node*)ptr_temp);
    }
};

void retrieveTree(const mxArray *ptr_mat_, QuadTreeFastPooled*& tree_)
{
    double* data = mxGetPr(ptr_mat_);
    if(data == nullptr)
        mexErrMsgTxt("vararg{1} must be a valid quadtreefastpooled pointer.\n");
    
    intptr_t tree_ptr_int = (intptr_t)data[0];
    tree_ = (QuadTreeFastPooled*)tree_ptr_int;

    if(tree_ == nullptr)
		mexErrMsgTxt("vararg{1} must be a valid quadtree pointer.\n");
};

void retrieveTree(
	const mxArray *ptr_mat_,
	QuadTreeFastMultiplePooled*& tree_)
{
	double* data = mxGetPr(ptr_mat_);
	if (data == nullptr)
		mexErrMsgTxt("vararg{1} must be a valid QuadTreeFastMultiplePooled pointer.\n");

	intptr_t tree_ptr_int = (intptr_t)data[0];
	tree_ = (QuadTreeFastMultiplePooled*)tree_ptr_int;

	if (tree_ == nullptr)
		mexErrMsgTxt("vararg{1} must be a valid quadtree pointer.\n");
};

void retrieveImage(const mxArray* ptr_mat_, 
double*& img_ptr, int& n_rows, int& n_cols) {
   // retrieve pointer from the MX form
   img_ptr = mxGetPr(ptr_mat_);
   // check that I actually received something
   if (img_ptr == NULL)
      mexErrMsgTxt("vararg{2} must be a [kxN] matrix of data.\n");
   // retrieve amount of points
   n_rows = (int)mxGetM(ptr_mat_);
   n_cols = (int)mxGetN(ptr_mat_); // [du1,du2,du3...;dv1, dv2, dv3...]
}

#endif