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
n_start = 116;
n_final = 2890;
n_key   = n_start;
n_cur   = n_start + 1;
data_info = loadStereoData(dataset_name, subset_name, dataset_dir, 0);
global objpool_node objpool_elem

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
len_min = 20; % 15 pixel �ּ� ����.
len_max = 40; % 40 pixel �ִ� ����.

% (5) iteration ����
global MAX_ITER thres_rot thres_trans 
MAX_ITER    = 50;
thres_rot   = 3; % degree
thres_trans = 0.4; % [m]

%% ù �̹��� �о����.
% key ���׷��� �̹����� �о�´�.
[img_l, img_r] = imreadStereo(data_info, n_start, flag_histeq);
% key ���׷��� �̹����� undistort & rectify ���ش�.
[img_l_rect, img_r_rect, data_info] = stereoRectifyUndist(img_l, img_r, data_info); % �� �Ǵ°ɱ�...
% key ���׷��� �̹����� gradient & edge & saliency ��� �����Ѵ�.
frame_k = frameConstruct(img_l_rect, img_r_rect, n_start); % �뷫 3mb�̴�.
% key static stereo ����
frame_k = stereoDisparityStatic(frame_k);
X_k    = frame_k.left.pts3d(:,find(frame_k.left.is_recon));
figure(); plot3(X_k(1,:),X_k(2,:),X_k(3,:),'r.','markersize',2);axis equal;

%% ���� �̹��� ���� �� ���� ����

figure();
him = imshow([img_l_rect,img_r_rect],[0,255]);hold on;
hplotl_1 = plot(0,0,'mx','markersize',2);
hplotl_2 = plot(0,0,'bo','markersize',2);
hplotr_1 = plot(0,0,'mx','markersize',2);
hplotr_2 = plot(0,0,'bo','markersize',2);
hline_l = line([zeros(1,5000);zeros(1,5000)], [zeros(1,5000);zeros(1,5000)]);
for k = 1:5000
   hline_l(k,1).Color = [1,1,1];
   hline_l(k,1).LineWidth = 1.5;
   hline_l(k,1).XData = [0,0];
   hline_l(k,1).YData = [0,0];
end
hline_r = line([zeros(1,5000);zeros(1,5000)], [zeros(1,5000);zeros(1,5000)]);
for k = 1:5000
   hline_r(k,1).Color = [1,1,1];
   hline_r(k,1).LineWidth = 1.5;
   hline_r(k,1).XData = [0,0];
   hline_r(k,1).YData = [0,0];
end

drawnow;

xi_ck = zeros(6,1);
K    = data_info.rectify.K;
Kinv = data_info.rectify.Kinv;
for n_img = n_start + 1:n_final
   [objpool_node, objpool_elem] = objectpoolBuild(262144); % objectpool�� �����.
   
   % ���� �̹��� ��ȣ.
   n_cur = n_img;
   % ���� ���׷��� �̹����� �о�´�.
   [img_l, img_r] = imreadStereo(data_info, n_cur, flag_histeq);
   % ���� ���׷��� �̹����� undistort & rectify ���ش�.
   [img_l_rect, img_r_rect, data_info] = stereoRectifyUndist(img_l, img_r, data_info); % �� �Ǵ°ɱ�...
   
   % ���� ���׷��� �̹����� gradient & edge & saliency ��� �����Ѵ�.
   frame_c = frameConstruct(img_l_rect, img_r_rect, n_cur); % �뷫 3mb�̴�.
   % ���� ���׷������� quadtree�� �����Ѵ�.
   qts_left  = quadtreemBuild(frame_c.left.pts_edge', double(frame_c.left.bins_edge(1,:)'), double(frame_c.left.bins_edge(2,:)'),...
      n_rows, n_cols, max_depth, eps, thres_dist, objpool_node, objpool_elem);
   qts_right = quadtreemBuild(frame_c.right.pts_edge', double(frame_c.right.bins_edge(1,:)'), double(frame_c.right.bins_edge(2,:)'),...
      n_rows, n_cols, max_depth, eps, thres_dist, objpool_node, objpool_elem);
   
   % k�� ���������� �ҷ���.
   mask_alive = frame_k.left.is_recon; % ���� ������ mask�� ����ִ�.
   X_k    = frame_k.left.pts3d;
   
   % Gauss-Newton iteration�� ���� �ظ� ��´�.
   r_prev = 1e15; r_curr = 0;

   for iter = 1:MAX_ITER
      % ��ȯ�� ���Ѵ�.
      g_ck = se3Exp(xi_ck);
      g_lr = data_info.extrin.g_nlnr;
      % X_k ������ �����Ѵ�.
      X_k_warp_l = g_ck(1:3,1:3)*X_k + g_ck(1:3,4);
      X_k_warp_r = g_lr(1:3,1:3)^-1*(X_k_warp_l - g_lr(1:3,4));
      % X_k ������ ���������Ѵ�.
      pts_k_warp_l = [K(1,1)*X_k_warp_l(1,:)./X_k_warp_l(3,:) + K(1,3);...
         K(2,2)*X_k_warp_l(2,:)./X_k_warp_l(3,:) + K(2,3)];
      pts_k_warp_r = [K(1,1)*X_k_warp_r(1,:)./X_k_warp_r(3,:) + K(1,3);...
         K(2,2)*X_k_warp_r(2,:)./X_k_warp_r(3,:) + K(2,3)];
      % ��Ī: iteration �� 1���� ũ�� Cached �� �Ѵ�.
      if(iter > 1)
         [idx_left, node_left] = quadtreemCachedNN(qts_left, pts_k_warp_l', double(frame_k.left.dirs(1,:)'),node_left);
         [idx_right, node_right] = quadtreemCachedNN(qts_right, pts_k_warp_r', double(frame_k.left.dirs(1,:)'),node_right);
      else
         [idx_left, node_left] = quadtreemNN(qts_left, pts_k_warp_l', double(frame_k.left.dirs(1,:)'));
         [idx_right, node_right] = quadtreemNN(qts_right, pts_k_warp_r', double(frame_k.left.dirs(1,:)'));
      end
      % mask update
      mask_alive(find((idx_left<0 ) && (idx_right<0))) = false;
      idx_alive  = find(mask_alive); % ���� ����ִ� ����.
      
      idx_left  = idx_left(idx_alive);
      idx_right = idx_right(idx_alive);
      pts_k_warp_l = pts_k_warp_l(:,idx_alive);
      pts_k_warp_r = pts_k_warp_r(:,idx_alive);
      
      n_pts_k = length(idx_alive); % ��ȿ�� k�� ����.
      Jt = zeros(6, 2*n_pts_k); % left - right ������ 2���.
      rt = zeros(1, 2*n_pts_k);
      
      % Jacobian�� residual �� ����Ѵ�.
      % (1) ��Ī�� ���� ��ǥ�� gradient�� ������.
      grad_c_l = [frame_c.left.grad_edge(1,idx_left)./frame_c.left.grad_edge(3,idx_left);...
         frame_c.left.grad_edge(2,idx_left)./frame_c.left.grad_edge(3,idx_left)];
      grad_c_r = [frame_c.right.grad_edge(1,idx_right)./frame_c.right.grad_edge(3,idx_right);...
         frame_c.right.grad_edge(2,idx_right)./frame_c.right.grad_edge(3,idx_right)];
      pts_c_l  = frame_c.left.pts_edge(:,idx_left);
      pts_c_r  = frame_c.right.pts_edge(:,idx_right);
      
      % (2) residual ���
      rt(1,1:n_pts_k) = grad_c_l(1,:).*(pts_k_warp_l(1,:) - pts_c_l(1,:)) + grad_c_l(2,:).*(pts_k_warp_l(2,:) - pts_c_l(2,:));
      rt(1,n_pts_k+1:2*n_pts_k) = grad_c_r(1,:).*(pts_k_warp_r(1,:) - pts_c_r(1,:)) + grad_c_r(2,:).*(pts_k_warp_r(2,:) - pts_c_r(2,:));
      % (3) Jacobian ���
      gfu = grad_c_l(1,:)*K(1,1);
      gfv = grad_c_l(2,:)*K(2,2);
      iz  = 1./X_k_warp_l(3,idx_alive);
      xiz = X_k_warp_l(1,idx_alive).*iz;
      yiz = X_k_warp_l(2,idx_alive).*iz;
      
      Jt(1,1:n_pts_k) = gfu.*iz;
      Jt(2,1:n_pts_k) = gfv.*iz;
      Jt(3,1:n_pts_k) = (-gfu.*xiz - gfv.*yiz).*iz;
      Jt(4,1:n_pts_k) = -gfu.*xiz.*yiz - gfv.*(1 + yiz.*yiz);
      Jt(5,1:n_pts_k) = gfu.*(1 + xiz.*xiz) + gfv.*xiz.*yiz;
      Jt(6,1:n_pts_k) = -gfu.*yiz + gfv.*xiz;
      
      gfu = grad_c_r(1,:)*K(1,1);
      gfv = grad_c_r(2,:)*K(2,2);
      iz  = 1./X_k_warp_r(3,idx_alive);
      xiz = X_k_warp_r(1,idx_alive).*iz;
      yiz = X_k_warp_r(2,idx_alive).*iz;
      
      Jt(1,n_pts_k+1:2*n_pts_k) = gfu.*iz;
      Jt(2,n_pts_k+1:2*n_pts_k) = gfv.*iz;
      Jt(3,n_pts_k+1:2*n_pts_k) = (-gfu.*xiz - gfv.*yiz).*iz;
      Jt(4,n_pts_k+1:2*n_pts_k) = -gfu.*xiz.*yiz - gfv.*(1 + yiz.*yiz);
      Jt(5,n_pts_k+1:2*n_pts_k) = gfu.*(1 + xiz.*xiz) + gfv.*xiz.*yiz;
      Jt(6,n_pts_k+1:2*n_pts_k) = -gfu.*yiz + gfv.*xiz;
      
      % Weight ����� �ʼ���. �߸� ��Ī�� �ֵ��� �ɷ����� ����.
      thres_huber = 2.5;
      index_huber = find(abs(rt)>thres_huber);
      W = ones(1,2*n_pts_k);
      W(1,index_huber) = thres_huber./abs(rt(index_huber));
      
      % (4) Hessian ���
      JtW = bsxfun(@times,Jt,W);
      H = JtW*Jt';
      % (5) delta_xi ���
      delta_xi = -H^-1*JtW*rt';
      % (6) update
      xi_ck = xi_ck + delta_xi;
      
      r_norm = sqrt(sum(rt(index_huber).^2)/length(index_huber)*2);
      
%       figure(153);
%       plot(rt);drawnow;
%       fprintf('ncur: %d / iter: %d / norm: %0.6f\n', n_img, iter, r_norm);
      fprintf('angle: %0.3f\n', 180/pi*norm(r2a(g_ck(1:3,1:3))));
   end
   
   % ���� update.
   % (1) X_k ����
   % (2) xi_ck �� X_k �� �̿��ؼ� X_k ���� �� ������Ʈ ����.
   % if(���� ���� O)
   % (2-a) static stereo �� temporal stereo -> �ռ�.
   % else(���� ���� X)
   % (2-b) temporal stereo only.
   
   
   % keyframe update ���� ����. 
   % roll > 4 degs && rotation > 5 degs && trans > 0.08 m
   
   him.CData = [img_l_rect,img_r_rect];
   hplotl_1.XData = pts_k_warp_l(1,:);
   hplotl_1.YData = pts_k_warp_l(2,:);
   hplotl_2.XData = pts_c_l(1,:);
   hplotl_2.YData = pts_c_l(2,:);
   
   hplotr_1.XData = pts_k_warp_r(1,:)+n_cols;
   hplotr_1.YData = pts_k_warp_r(2,:);
   hplotr_2.XData = pts_c_r(1,:)+n_cols;
   hplotr_2.YData = pts_c_r(2,:);
   
   for k = 1:n_pts_k
      hline_l(k,1).Color = [0,1,0];
      hline_l(k,1).LineWidth = 2;
      hline_l(k,1).XData = [pts_k_warp_l(1,k);pts_c_l(1,k)];
      hline_l(k,1).YData = [pts_k_warp_l(2,k);pts_c_l(2,k)];
   end
   for k = 1:n_pts_k
      hline_r(k,1).Color = [0,1,0];
      hline_r(k,1).LineWidth = 2;
      hline_r(k,1).XData = [pts_k_warp_r(1,k)+n_cols;pts_c_r(1,k)+n_cols];
      hline_r(k,1).YData = [pts_k_warp_r(2,k);pts_c_r(2,k)];
   end
   drawnow;

   quadtreemDelete(qts_left);
   quadtreemDelete(qts_right);
   objectpoolDelete(objpool_node, objpool_elem);
end

%%
% for i = 1:100
%    fprintf('i: %d\n',i);
%    [objpool_node, objpool_elem] = objectpoolBuild(131072); % objectpool�� �����.
%    qts_left  = quadtreemBuild(frame_c.left.pts_edge', double(frame_c.left.bins_edge(1,:)'), double(frame_c.left.bins_edge(2,:)'),...
%       n_rows, n_cols, max_depth, eps, thres_dist, objpool_node, objpool_elem);
%    qts_right = quadtreemBuild(frame_c.right.pts_edge', double(frame_c.right.bins_edge(1,:)'), double(frame_c.right.bins_edge(2,:)'),...
%       n_rows, n_cols, max_depth, eps, thres_dist, objpool_node, objpool_elem);
%    
%    quadtreemDelete(qts_left);
%    quadtreemDelete(qts_right);
%    objectpoolDelete(objpool_node, objpool_elem);
% end
