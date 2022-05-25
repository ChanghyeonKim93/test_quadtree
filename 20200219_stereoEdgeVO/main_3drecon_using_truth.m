close all;clear all;clc;
addpath('C:\dev\mexopencv');
addpath('C:\dev\mexopencv\opencv_contrib');
addpath('functions_transform');

fprintf('===========Algorithm start============\n');
profile on;
% cd('C:\dev\mexopencv')
% mexopencv.make('opencv_path','C:\dev\build\install', 'opencv_contrib',true)

addpath('functions_mex');
% mexbuild; % mex �Լ����� �ѹ��� compile
%%
% (1) �����ͼ� �ε� ����
global dataset_dir dataset_name subset_name
dataset_dir  = 'F:\#����\#DATASET';
% dataset_dir  = 'G:\#����';
dataset_name = 'euroc'; % 'euroc' 'custom' 'kitti'
subset_name  = 'V1_01_easy';

% (2) flag��.
global flag_draw flag_histeq
flag_draw    = true;
flag_histeq  = false;

% (3) �����ͼ� �о����, ��� �� �̹��� ���� ���� (data_info�� ���ƾ��� ������)
global data_info n_start n_final n_cur n_key
n_start = 500;
% n_final = 2890;
n_final = 530;
n_key   = n_start;
n_cur   = n_start + 1;
data_info = loadStereoData(dataset_name, subset_name, dataset_dir, 0);
% global objpool_node objpool_elem
% [objpool_node, objpool_elem] = objectpoolBuild(131072); % objectpool�� �����.

%%
% (4) quadtree �� ����. ������ current image�� left & right tree�� ����.
global qts_left qts_right idx_left idx_right node_left node_right
global thres_dist max_depth eps
qts_left = -1; % ���� �̹����� ���� Ʈ�� �ּ�
qts_right = -1; % ���� �̹����� ������ Ʈ�� �ּ�

idx_left = []; % left �̹����� ���� matching �� ���� ��ȣ.
idx_right = []; % right �̹����� ���� matching �� ���� ��ȣ.
node_left = []; % left �̹����� ���� cached �� node��
node_right = []; % right �̹����� ���� cached �� ndoe��
thres_dist = 30; % 30 px �̻� ��Ī�Ǹ� ������.
max_depth = 5; % �ְ� ����.
eps = 0.0; % epsilon .

n_rows = data_info.intrin.n_rows.left;
n_cols = data_info.intrin.n_cols.left;

% (5) edge ���� ����
global thres_grad_min thres_grad_max ratio_reduction_right size_sobel overlap len_min len_max
thres_grad_min = 55;
thres_grad_max = 112;
ratio_reduction_right = 0.8; % right �̹����� �ִ��� left���� ���̴� ��� ���� �����ϱ� ���� 0.8�� ���ϰ� threshold ����.
size_sobel = 3;
overlap = 15;
len_min = 3; % 15 pixel �ּ� ����.
len_max = 5; % 40 pixel �ִ� ����.

%% ù �̹��� �о����.
% key ���׷��� �̹����� �о�´�.
[img_l, img_r] = imreadStereo(data_info, n_start, flag_histeq);
% key ���׷��� �̹����� undistort & rectify ���ش�.
[img_l_rect, img_r_rect, data_info] = stereoRectifyUndist(img_l, img_r, data_info); % �� �Ǵ°ɱ�...

% key ���׷��� �̹����� gradient & edge & saliency ��� �����Ѵ�.
frame_k = frameConstruct(img_l_rect, img_r_rect, n_start); % �뷫 3mb�̴�.
% key static stereo ����
frame_k = stereoDisparityStatic(frame_k);
drawValidPoints(frame_k);

figure();imshow([frame_k.left.img,frame_k.right.img],[0,255]);
figure();imshow([frame_k.left.edge(:,:,1)>-1,frame_k.right.edge(:,:,1)>-1],[0,1]);

K    = data_info.rectify.K;
Kinv = data_info.rectify.Kinv;
profile on;
%% ���� �̹��� ���� �� ���� ����
xi_ck = zeros(6,1);
g_k = data_info.gt.g(:,:,n_start);
g_c = ones(4,4);
for n_img = n_start + 1:1:n_final
   % ���� �̹��� ��ȣ.
   n_cur = n_img;
   % ���� �ڼ�
   g_c = data_info.gt.g(:,:,n_cur);
   % ���� ���׷��� �̹����� �о�´�.
   [img_l, img_r] = imreadStereo(data_info, n_cur, flag_histeq);
   % ���� ���׷��� �̹����� undistort & rectify ���ش�.
   [img_l_rect, img_r_rect, data_info] = stereoRectifyUndist(img_l, img_r, data_info); % �� �Ǵ°ɱ�...
   % ���� ���׷��� �̹����� gradient & edge & saliency ��� �����Ѵ�.
   frame_c = frameConstruct(img_l_rect, img_r_rect, n_cur); % �뷫 3mb�̴�.
   % current static stereo ����
%    frame_c = stereoDisparityStatic(frame_c);
   
   % overlap ratio ����ϰ�, keyframe ���� warp��.
   g_ck = g_c^-1*g_k;
   ratio_overlap = calcOverlapRatio(frame_k, g_ck);
   fprintf('overlap: %0.3f\n',ratio_overlap);
   
   % frame_c �� �̿��ؼ� static update ����. (��, 2 cm �̻� �̵� & 0.5�� �̻�ȸ��)
   if(norm(g_ck(1:3,4)) > 0.02 && norm( r2a(g_ck(1:3,1:3)) ) >= 0.1 /180*pi)
      % warp ��, frame_c���� static stereo ����.
%       frame_k = stereoDisparityStaticWarped(frame_c, frame_k, g_ck);
      
      % static stereo ����� frame_k�� �Ű���.
      frame_k = stereoDisparityTemporal(frame_k, frame_c, g_ck);
      
%       drawValidPoints(frame_k);
      % frame_c �� �̿��ؼ� temporal update ����. (�� 2 cm �̻� �̵� ��)
      fprintf('another static on: %d\n',n_img);
   end
      
   %    % update keyframe to , center�� ������ 30 �� �̻� �������� ������Ʈ.
   %    if(ratio_overlap <= 0.7 & norm(g_k(1:3,4)-g_c(1:3,4))>0.15)
   %       X_c    = frame_c.left.pts3d(:,find(frame_c.left.is_recon));
   %       X_c = data_info.gt.g(1:3,1:3,n_img)*X_c + data_info.gt.g(1:3,4,n_img);
   %       plot3(X_c(1,:),X_c(2,:),X_c(3,:),'r.','markersize',1);
   %       drawnow;
   %       frame_k = frame_c;
   %
   %    end
end

profile viewer;