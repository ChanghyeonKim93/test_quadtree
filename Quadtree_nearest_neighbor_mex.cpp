#include "QuadTree.h"
#include "mex.h"

void retrieve_tree( const mxArray* matptr, QuadTree* & tree){
    // retrieve pointer from the MX form
    double* pointer0 = mxGetPr(matptr);
    // check that I actually received something
    if( pointer0 == NULL )
        mexErrMsgTxt("vararg{1} must be a valid Quad tree pointer\n");
    // convert it to "long" datatype (good for addresses)
    intptr_t pointer1 = (intptr_t) pointer0[0];
    // convert it to "QuadTree Node*"
    tree = (QuadTree*) pointer1;
    // check that I actually received something
    if( tree == NULL )
        mexErrMsgTxt("vararg{1} must be a valid quadtree pointer\n");
}

void retrieve_query( const mxArray* matptr, double*& data, int& npoints, int& ndims){	
	// retrieve pointer from the MX form
    data = mxGetPr(matptr);
    // check that I actually received something
    if( data == NULL )
        mexErrMsgTxt("vararg{2} must be a [kxN] matrix of data\n");
    // retrieve amount of points
    npoints = mxGetM(matptr);
    ndims   = mxGetN(matptr);
}

void retrieve_cache( const mxArray* matptr, double*& data){	
	// retrieve pointer from the MX form
    data = mxGetPr(matptr);
    // check that I actually received something
    if( data == NULL )
        mexErrMsgTxt("vararg{3} must be a cache array.\n");
}




void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[]){
	// check number of arguments
	if( nrhs!=3 )
		mexErrMsgTxt("This function requires 3 arguments\n");
	if( !mxIsNumeric(prhs[0]) )
		mexErrMsgTxt("varargin{0} must be a valid quadtree root node pointer\n");
	if( !mxIsNumeric(prhs[1]) )
		mexErrMsgTxt("varargin{1} must be a query set of points\n");
		
    // retrieve the tree pointer
    QuadTree* tree;
    retrieve_tree( prhs[0], tree ); 
    // retrieve the query data
    double* query_data;
    double* cached_data;
    int npoints, ndims;
    retrieve_query( prhs[1], query_data, npoints, ndims );
    // printf("query size: %dx%d\n", npoints, ndims);

    // retrieve the cached nodes.
    retrieve_cache( prhs[2], cached_data);
    // check dimensions

    // npoints x 1 indexes in output
    plhs[0] = mxCreateDoubleMatrix(npoints, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(npoints, 1, mxREAL);
    double* indexes = mxGetPr(plhs[0]);
    double* cached_data_new = mxGetPr(plhs[1]);
    
    // execute the query FOR EVERY point storing the index
    std::vector<Point> qs;
    std::vector<void*> cached_nodes;
    for(int i =0; i<npoints; i++){
        qs.push_back(Point(query_data[i],query_data[i+npoints],i));
        cached_nodes.push_back((void*)((intptr_t)cached_data[i]));
    }

    std::vector<int> indexVec;
    indexVec.reserve(0);
    tree->cachedNNSearchMultiple(qs, cached_nodes, indexVec);
    /*for(int i=0; i<npoints; i++)
    {
        //printf("%d\n",indexVec[i]);
        indexes[i] = indexVec[i]+1; //M-idx
        cached_data_new[i] = (intptr_t)cached_nodes[i];
    }*/
}