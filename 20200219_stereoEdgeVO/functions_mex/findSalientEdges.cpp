/*	mex_findEdgeDirAndSalientAndPixels.cpp
*	Examples:
*      output_image_ptr = mex_findEdgeDirAndSalientAndPixels(input_image_ptr);		
*		
*  Notes:
*
*  To Build:
*      mex -v mex_findEdgeDirAndSalientAndPixels.cpp
*
*	Author:
*		Changhyeon Kim
*		rlackd93@snu.ac.kr / hyun91015@gmail.com
*      02/Sep/2019
*/
#include <iostream>
#include <vector>
#include <stack>
#include <cmath>

#include "mex.h"
#include "matrix.h" //isNaN/isinf

#include "CommonStruct.h"
#include "mex_io.h"

void floodFillStack(Point2<int>& pt_q, short& dir_q, int& max_length, int& n_rows, int& n_cols, short* dir1_img, short* dir2_img, std::vector<Point2<int>>& edgelet);
inline void calcMeans(std::vector<Point2<int>>& pts, Point2<double>& pt_mean);
inline void calcMeanAndPrincipleAxis(std::vector<Point2<int>>& pts, Point2<double>& pt_mean, double& evalsqr_ratio, Point2<double>& evec);

void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[]) {
   // check number of arguments
   // 입력: (1) edge_img, (2)du_img, (3)dv_img, (4) overlap_angle, (5)min_len, (6)max_len
   // 출력: (1) pts_edge(2xN), (2) bins_edge(2xN), (3) dudv_edge (2xN), 
   // (4) dir1_img, (5) dir2_img. (6) edgelets

   if (nrhs != 6)
      mexErrMsgTxt("This function requires 6 arguments\n");
   if (!mxIsNumeric(prhs[0]))
      mexErrMsgTxt("input 1 must be a double image.\n");
   if (!mxIsNumeric(prhs[1]))
      mexErrMsgTxt("input 2 must be a double image.\n");
   if (!mxIsNumeric(prhs[2]))
      mexErrMsgTxt("input 3 must be a double image.\n");

   // retrieve the query data
   double* edge_img = NULL;
   double* du_img   = NULL;
   double* dv_img   = NULL;
   int n_rows, n_cols;
   int n_elem;

   // load inputs: edge_img (480 x 640), du_img (480 x 640), dv_img (480 x 640)
   retrieveImage(prhs[0], edge_img, n_rows, n_cols); // edge 
   retrieveImage(prhs[1], du_img,   n_rows, n_cols); // du
   retrieveImage(prhs[2], dv_img,   n_rows, n_cols); // dv
   n_elem = n_rows*n_cols; // 총 원소갯수.
   // printf("n_rows: %d, n_cols: %d, n_elem: %d\n", n_rows, n_cols, n_elem);

   double overlap = (double)(*mxGetPr(prhs[3]));
   if(overlap < 0)
      mexErrMsgTxt("Overlap should be larger than 0(double).\n"); 
   else if(overlap > 22.5)
      mexErrMsgTxt("Overlap should be smaller than 22.5 degrees.\n"); 

   int min_length = (int)(*mxGetPr(prhs[4]));
   if(min_length < 0)
      mexErrMsgTxt("min_length needs to be larger than 0(integer).\n");
   int max_length = (int)(*mxGetPr(prhs[5]));
   if(max_length < 0)
      mexErrMsgTxt("min_length needs to be larger than 0(integer).\n");  
   if(max_length < min_length)
      mexErrMsgTxt("max_length needs to be larger than min_length(integer).\n"); 
   
   // 1. Classify direction and n_pts
   int fbit  = 10; // bit shifter
   
   int t225  = round(tan(22.5*D2R)*1024);
   int t225p = round(tan((22.5+overlap)*D2R)*1024);
   int t225m = round(tan((22.5-overlap)*D2R)*1024);
   int t675  = round(tan(67.5*D2R)*1024);
   int t675p = round(tan((67.5+overlap)*D2R)*1024);
   int t675m = round(tan((67.5-overlap)*D2R)*1024);
      
   int du   = -1, dv = -1;
   int slp  = -1; // slope
   int bin1 = -1, bin2 = -1;

   int ind = 0;
   int n_pts_edge = 0;

   std::vector<Point2<int>> pts_edge;
   std::vector<int> grad_u; 
   std::vector<int> grad_v;
   std::vector<int> bins1;
   std::vector<int> bins2;
   grad_u.reserve(30000);
   grad_v.reserve(30000);
   pts_edge.reserve(30000);
   bins1.reserve(30000);
   bins2.reserve(30000);

   short* dir1_img = new short[n_elem];
   short* dir2_img = new short[n_elem];
   for(int i = 0; i<n_elem; i++){
      dir1_img[i] = -1;
      dir2_img[i] = -1;
   }

   for(int u = 1; u <n_cols - 1; u++){
      ind = u*n_rows; // MATLAB index.
      // ind = v*ncols + u; // C++ index.
      for(int v = 1; v < n_rows - 1; v++){
         ++ind;
         if(edge_img[ind]) { // edge 픽셀인 경우에만.
            ++n_pts_edge; // edge 픽셀 갯수 증가.
            du = (int)du_img[ind];
            dv = (int)dv_img[ind];
            bin1 = -1;
            bin2 = -1;
            
            if(du > 0) {
               slp = (dv<<fbit)/du;
               if(dv > 0) {
                  if(slp < t225){
                     bin1 = 0; if(slp > t225m) bin2 = 1;
                  }
                  else if(slp < t675){
                     bin1 = 1;
                     if(slp < t225p) bin2 = 0;
                     else if(slp > t675m) bin2 = 2;
                  }
                  else {
                     bin1 = 2;
                     if(slp < t675p) bin2 = 1;
                  }
               } // end if(dv > 0)
               else { // dv <=0
                  if(slp > -t225){
                     bin1 = 0; if(slp < -t225m) bin2 = 7;
                  }
                  else if(slp > -t675){
                     bin1 = 7;
                     if(slp > -t225p) bin2 = 0;
                     else if(slp < -t675m) bin2 = 6;
                  }
                  else{
                     bin1 = 6; if(slp > -t675p) bin2 = 7;
                  }
               } // end else 
            } // end if(du > 0)
            else if(du < 0){
               slp = (dv<<fbit)/du;
               if(dv > 0){
                  if(slp < -t675){
                     bin1 = 2; if(slp > -t675p) bin2 = 3;
                  }
                  else if(slp < -t225){
                     bin1 = 3;
                     if(slp < -t675m) bin2 = 2;
                     else if(slp > -t225p) bin2 = 4;
                  }
                  else {
                     bin1 = 4; if(slp < -t225m) bin2 = 3;
                  }
               }
               else { // dv <=0
                  if(slp <t225){
                     bin1 = 4; if(slp > t225m) bin2 = 5;
                  }
                  else if(slp <t675){
                     bin1 = 5;
                     if(slp < t225p) bin2 = 4;
                     else if(slp > t675m) bin2 = 6;
                  }
                  else{
                     bin1 = 6; if(slp < t675p) bin2 = 5;
                  }
               }
            }
            else { // du == 0
               if(dv > 0) bin1 = 2;
               else if(dv < 0) bin1 = 6;
               else bin1 = -1; // 기울기가 아예 0인경우다 ; 
            }

            // 데이터 정리.
            pts_edge.push_back(Point2<int>(u,v));
            bins1.push_back(bin1);
            bins2.push_back(bin2);
            grad_u.push_back(du);
            grad_v.push_back(dv);

            // dir1_img, dir2_img에 넣어준다.
            dir1_img[ind] = bin1;
            dir2_img[ind] = bin2;
         }
      }
   }

   // 출력 할 dir1_img dir2_img를 복사한다.
   double* dir1_img_copy = new double[n_elem]();
   double* dir2_img_copy = new double[n_elem]();

   // initialize img_dir1_copy
   for (int i = 0; i < n_elem; i++) {
	   *(dir1_img_copy + i) = *(dir1_img + i);
	   *(dir2_img_copy + i) = *(dir2_img + i);
   }

   // printf("# of edge pixels: %d\n", n_pts_edge);

   // 2. Find salient edgelets
   int query_dirs[8][3] = {{0,1,7},{1,0,2},{2,1,3},{3,2,4},{4,3,5},{5,4,6},{6,5,7},{7,6,0}};
   std::vector<std::vector<Point2<int>>> edgelets;
   std::vector<double> eval_ratios;
   std::vector<Point2<int>> edgelet;
   edgelets.reserve(4000); // 총 4000개의 edgelets을 담는다.
   edgelet.reserve(max_length);
   eval_ratios.reserve(4000);

   std::vector<Point2<double>> pts_centers;
   std::vector<int> bins_centers;
   pts_centers.reserve(4000);
   bins_centers.reserve(4000);

   Point2<int> pt_q;
   short dir_q;
   for(int i = 0; i < pts_edge.size(); i++) {
      pt_q = pts_edge[i];
      ind  = pt_q.u*n_rows + pt_q.v; // MATLAB index, C++ index ind = v*n_cols + u;

      // 엣지가 있는 경우,
      if(dir1_img[ind] > -1)
      {
         dir_q  = dir1_img[ind];
         edgelet.resize(0);

         // find edgelet by queue-based DFS.
         floodFillStack(pt_q, dir_q, max_length, n_rows, n_cols, dir1_img, dir2_img, edgelet);

         // 최소 길이 조건을 만족 한 경우.
         if(edgelet.size() > min_length)
         {
            // 중심점과 주요 방향을 계산함.
            Point2<double> pt_mean;
            Point2<double> evec;
            double evalsqr_ratio;

            // 주요 방향을 계산. (Eigenvalue decomposition for symmetric matrix)
            calcMeanAndPrincipleAxis(edgelet, pt_mean, evalsqr_ratio, evec);
            
            pt_mean.v = round(pt_mean.v);

            pts_centers.push_back(pt_mean);
            bins_centers.push_back(dir_q);
            eval_ratios.push_back(evalsqr_ratio);
               
            edgelets.push_back(edgelet);
         }
      }
   }
 
   // 출력 : edge points 좌표 / bin1bin2 한꺼번에 
   plhs[0] = mxCreateDoubleMatrix(2, n_pts_edge, mxREAL);
   plhs[1] = mxCreateDoubleMatrix(2, n_pts_edge, mxREAL);
   plhs[2] = mxCreateDoubleMatrix(3, n_pts_edge, mxREAL);
   double* plhs1_ptr = mxGetPr(plhs[0]);
   double* plhs2_ptr = mxGetPr(plhs[1]);
   double* plhs3_ptr = mxGetPr(plhs[2]);
   for(int i = 0; i<n_pts_edge; i++){
      int ind = 2*i;
      *(plhs1_ptr + ind) = pts_edge[i].u + 1;
      *(plhs1_ptr + ind + 1) = pts_edge[i].v + 1;
      *(plhs2_ptr + ind) = bins1[i];
      *(plhs2_ptr + ind + 1) = bins2[i];

      *(plhs3_ptr + 3*i) = grad_u[i];
      *(plhs3_ptr + 3*i + 1) = grad_v[i];
      *(plhs3_ptr + 3*i + 2) = std::abs(grad_u[i]) + std::abs(grad_v[i]);
   }
   plhs[3] = mxCreateDoubleMatrix(n_rows, n_cols, mxREAL);
   plhs[4] = mxCreateDoubleMatrix(n_rows, n_cols, mxREAL);
   double* dir1_img_output_ptr = mxGetPr(plhs[3]);
   double* dir2_img_output_ptr = mxGetPr(plhs[4]);
   for (int i = 0; i < n_elem; i++) {
	   *(dir1_img_output_ptr + i) = *(dir1_img_copy + i);
	   *(dir2_img_output_ptr + i) = *(dir2_img_copy + i);
   }

   int n_field = 5;
   const char* field_names[] = { "n_edgelets","edgelets","pts_centers","bins_centers","evalratios"};
   plhs[5] = mxCreateStructMatrix(1, 1, n_field, field_names);
   mxArray* mx_n_edgelets = mxCreateDoubleScalar((double)(edgelets.size()));
   mxArray* mx_edgelets = mxCreateCellMatrix(1, edgelets.size());
   mxArray* mx_pts_centers = mxCreateDoubleMatrix(2, edgelets.size(), mxREAL);
   mxArray* mx_bins_centers = mxCreateDoubleMatrix(1, edgelets.size(),mxREAL);
   mxArray* mx_evals = mxCreateDoubleMatrix(1, edgelets.size(),mxREAL);

   double* ptr_pts_centers = mxGetPr(mx_pts_centers);
   double* ptr_bins_centers = mxGetPr(mx_bins_centers);
   double* ptr_evals = mxGetPr(mx_evals);


   for (int i = 0; i < edgelets.size(); i++) {
	   int n_pts_edgelet = edgelets[i].size();
	   double* pt_temp = (double*)mxMalloc(2 * n_pts_edgelet*sizeof(double));

      // edgelets
	   for (int j = 0; j < n_pts_edgelet; j++) {
		   *(pt_temp + 2 * j)     = (double)(edgelets[i][j].u + 1);
		   *(pt_temp + 2 * j + 1) = (double)(edgelets[i][j].v + 1);
	   }

      // pts_centers
      *(ptr_pts_centers+2*i)   = pts_centers[i].u + 1;
      *(ptr_pts_centers+2*i+1) = pts_centers[i].v + 1;
      *(ptr_bins_centers + i)  = bins_centers[i];
      *(ptr_evals + i) = eval_ratios[i];

	   mxArray* temp = mxCreateNumericMatrix(0, 0, mxDOUBLE_CLASS, mxREAL);
	   mxSetData(temp, pt_temp);

	   mxSetM(temp, 2);
	   mxSetN(temp, n_pts_edgelet);

	   mxSetCell(mx_edgelets, i, temp);
   }

   mxSetField(plhs[5], 0, "n_edgelets", mx_n_edgelets);
   mxSetField(plhs[5], 0, "edgelets", mx_edgelets);
   mxSetField(plhs[5], 0, "pts_centers", mx_pts_centers);
   mxSetField(plhs[5], 0, "bins_centers", mx_bins_centers);
   mxSetField(plhs[5], 0, "evalratios", mx_evals);

   
   delete[] dir1_img;
   delete[] dir2_img;
   delete[] dir1_img_copy;
   delete[] dir2_img_copy;
};



void floodFillStack(Point2<int>& pt_q, short& dir_q, int& max_length, int& n_rows, int& n_cols, short* dir1_img, short* dir2_img, std::vector<Point2<int>>& edgelet)
{
   edgelet.resize(0);
   std::stack<Point2<int>> st;
   st.push(pt_q);

   int u, v, ind, cnt;
   cnt = 0;
   while( !st.empty() ) {
      Point2<int> pt_temp = st.top();
      st.pop();      
      u = pt_temp.u;
      v = pt_temp.v;
      // 해당 원소를 넣는다.
      edgelet.push_back(Point2<int>(u, v));
      ++cnt;
      if(cnt < max_length && u > 0 && u < n_cols && v > 0 && v < n_rows){
          ind = u*n_rows + v - 1;
         // 1. (u, v - 1)
         if(dir1_img[ind] == dir_q || dir2_img[ind] == dir_q){
            dir1_img[ind] = -1;
			dir2_img[ind] = -1;

            st.push(Point2<int>(u, v-1));
         }
         ind = u*n_rows + v + 1;
         // 2. (u, v + 1)
         if(dir1_img[ind] == dir_q || dir2_img[ind] == dir_q){
            dir1_img[ind] = -1;
			dir2_img[ind] = -1;
            st.push(Point2<int>(u, v+1));
         }
         // 3. (u - 1, v) 
         ind = (u - 1)*n_rows + v;
         if(dir1_img[ind] == dir_q || dir2_img[ind] == dir_q){
            dir1_img[ind] = -1;
			dir2_img[ind] = -1;

            st.push(Point2<int>(u - 1, v));
         }
         // 4. (u + 1, v)
         ind = (u + 1)*n_rows + v;
         if(dir1_img[ind] == dir_q || dir2_img[ind] == dir_q){
            dir1_img[ind] = -1;
			dir2_img[ind] = -1;

            st.push(Point2<int>(u + 1, v));
         }

         // 5. (u - 1, v - 1)
         ind = (u - 1)*n_rows + v - 1;
         if(dir1_img[ind] == dir_q || dir2_img[ind] == dir_q){
            dir1_img[ind] = -1;
			dir2_img[ind] = -1;

            st.push(Point2<int>(u - 1, v - 1));
         }
         // 6. (u - 1, v + 1)
         ind = (u - 1)*n_rows + v + 1;
         if(dir1_img[ind] == dir_q || dir2_img[ind] == dir_q){
            dir1_img[ind] = -1;
			dir2_img[ind] = -1;

            st.push(Point2<int>(u - 1, v + 1));
         }
         // 7. (u + 1, v - 1)
         ind = (u + 1)*n_rows + v - 1;
         if(dir1_img[ind] == dir_q || dir2_img[ind] == dir_q){
            dir1_img[ind] = -1;
			dir2_img[ind] = -1;

            st.push(Point2<int>(u + 1, v - 1));
         }
         // 8. (u + 1, v + 1)
         ind = (u + 1)*n_rows + v + 1;
         if(dir1_img[ind] == dir_q || dir2_img[ind] == dir_q){
            dir1_img[ind] = -1;
			dir2_img[ind] = -1;

            st.push(Point2<int>(u + 1, v + 1));
         }

      }
   }
};

inline void calcMeans(std::vector<Point2<int>>& pts, Point2<double>& pt_mean){
   int i = 0;
   int sum_u = 0;
   int sum_v = 0; 
   for(i = 0; i < pts.size(); i++){
      sum_u += pts[i].u;
      sum_v += pts[i].v;
   }
   pt_mean.u = (double)sum_u / (pts.size());
   pt_mean.v = (double)sum_v / (pts.size());
};

inline void calcMeanAndPrincipleAxis(std::vector<Point2<int>>& pts, Point2<double>& pt_mean, double& evalsqr_ratio, Point2<double>& evec){
   int m_u = 0;
   int m_v = 0; 
   int m_uu = 0; 
   int m_vv = 0;
   int m_uv = 0;

   double Ninv = 1.0/(double)pts.size();
   double Ninv2 = Ninv*Ninv;
   for(int i = 0; i < pts.size(); i++){
      m_u += pts[i].u;
      m_v += pts[i].v;
      m_uu += pts[i].u*pts[i].u;
      m_uv += pts[i].u*pts[i].v;
      m_vv += pts[i].v*pts[i].v;
   }

   pt_mean.u = (double)m_u*Ninv;
   pt_mean.v = (double)m_v*Ninv;

   double a,b,c;
   a = (double)(m_uu*Ninv- m_u*m_u*Ninv2);
   b = (double)(m_uv*Ninv - m_u*m_v*Ninv2);
   c = (double)(m_vv*Ninv - m_v*m_v*Ninv2);

   double disk, lam1, lam2; // lam1이 large eigenvalue
   disk = sqrt((a-c)*(a-c) + 4*b*b);
   lam1 = 0.5*(a+c+disk);
   lam2 = 0.5*(a+c-disk);

   evalsqr_ratio = lam2/lam1; // small / large. 클 수록 linearity가 큰 것. 1에 가까우면 원에 가까움.

   double invnorm_evec = 1.0/sqrt(b*b + (a-lam1)*(a-lam1));
   evec.u = -b;
   evec.v = a-lam1;

   evec.u *= invnorm_evec;
   evec.v *= invnorm_evec;   
};