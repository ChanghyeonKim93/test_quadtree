close all;clear all;clc;
addpath('C:\dev\mexopencv');
addpath('C:\dev\mexopencv\opencv_contrib');
addpath('functions_transform');

fprintf('===========Algorithm start============\n');
profile on;
% cd('C:\dev\mexopencv')
% mexopencv.make('opencv_path','C:\dev\build\install', 'opencv_contrib',true)

addpath('functions_mex');
% mexbuild; % mex 함수들을 한번에 compile
%%
% (1) 데이터셋 로딩 관련
global dataset_dir dataset_name subset_name
% dataset_dir  = 'F:\#연구\#DATASET';
dataset_dir  = 'G:\#연구';
dataset_name = 'euroc'; % 'euroc' 'custom' 'kitti'
subset_name  = 'V1_01_easy';

% (2) flag들.
global flag_draw flag_histeq
flag_draw    = true;
flag_histeq  = false;

% (3) 데이터셋 읽어오고, 사용 할 이미지 범위 지정 (data_info를 갈아엎고 싶은데)
global data_info n_start n_final n_cur n_key
n_start = 116;
n_final = 2890;
n_key   = n_start;
n_cur   = n_start + 1;
data_info = loadStereoData(dataset_name, subset_name, dataset_dir, 0);
global objpool_node objpool_elem
[objpool_node, objpool_elem] = objectpoolBuild(131072); % objectpool을 만든다.


%%
% (4) quadtree 를 저장. 어차피 current image의 left & right tree만 쓴다.
global qts_left qts_right idx_left idx_right node_left node_right
global thres_dist max_depth eps
qts_left = -1; % 현재 이미지의 왼쪽 트리 주소
qts_right = -1; % 현재 이미지의 오른쪽 트리 주소

idx_left = []; % left 이미지에 대해 matching 된 점의 번호.
idx_right = []; % right 이미지에 대해 matching 된 점의 번호.
node_left = []; % left 이미지에 대해 cached 된 node들
node_right = []; % right 이미지에 대해 cached 된 ndoe들
thres_dist = 30; % 30 px 이상 매칭되면 버린다.
max_depth = 5; % 최고 깊이.
eps = 0.0; % epsilon .

n_rows = data_info.intrin.n_rows.left;
n_cols = data_info.intrin.n_cols.left;

% (5) edge 추출 관련
global thres_grad_min thres_grad_max ratio_reduction_right size_sobel overlap len_min len_max
thres_grad_min = 55;
thres_grad_max = 112;
ratio_reduction_right = 0.8; % right 이미지는 최대한 left에서 보이는 모든 점을 포착하기 위해 0.8배 약하게 threshold 설정.
size_sobel = 3;
overlap = 15;
len_min = 15; % 15 pixel 최소 추출.
len_max = 40; % 40 pixel 최대 추출.

%% 첫 이미지 읽어오기.
% key 스테레오 이미지를 읽어온다.
[img_l, img_r] = imreadStereo(data_info, n_start, flag_histeq);
% key 스테레오 이미지를 undistort & rectify 해준다.
[img_l_rect, img_r_rect, data_info] = stereoRectifyUndist(img_l, img_r, data_info); % 잘 되는걸까...
% key 스테레오 이미지의 gradient & edge & saliency 모두 추출한다.
frame_k = frameConstruct(img_l_rect, img_r_rect, n_start); % 대략 3mb이다.
% key static stereo 복원
frame_k = stereoDisparityStatic(frame_k);
X_k    = frame_k.left.pts3d(:,find(frame_k.left.is_recon));
figure(); plot3(X_k(1,:),X_k(2,:),X_k(3,:),'r.','markersize',2);hold on;axis equal;

K    = data_info.rectify.K;
Kinv = data_info.rectify.Kinv;
%% 연속 이미지 정렬 및 깊이 복원
xi_ck = zeros(6,1);
g_k = data_info.gt.g(:,:,n_start);
g_c = ones(4,4);
for n_img = n_start + 1:30:n_final
   % 현재 이미지 번호.
   n_cur = n_img;
   % 현재 자세
   g_c = data_info.gt.g(:,:,n_cur);
   
   % update keyframe to
   if(norm(g_k(1:3,4)-g_c(1:3,4))>0.15)
      % 현재 스테레오 이미지를 읽어온다.
      [img_l, img_r] = imreadStereo(data_info, n_cur, flag_histeq);
      % 현재 스테레오 이미지를 undistort & rectify 해준다.
      [img_l_rect, img_r_rect, data_info] = stereoRectifyUndist(img_l, img_r, data_info); % 잘 되는걸까...
      % 현재 스테레오 이미지의 gradient & edge & saliency 모두 추출한다.
      frame_c = frameConstruct(img_l_rect, img_r_rect, n_cur); % 대략 3mb이다.
      % current static stereo 복원
      frame_c = stereoDisparityStatic(frame_c);
      X_c    = frame_c.left.pts3d(:,find(frame_c.left.is_recon));
      X_c = data_info.gt.g(1:3,1:3,n_img)*X_c + data_info.gt.g(1:3,4,n_img);
      
      plot3(X_c(1,:),X_c(2,:),X_c(3,:),'r.','markersize',1);
      drawnow;
      frame_k = frame_c;
      
   end
end