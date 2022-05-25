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
dataset_dir  = 'F:\#연구\#DATASET';
% dataset_dir  = 'G:\#연구';
dataset_name = 'euroc'; % 'euroc' 'custom' 'kitti'
subset_name  = 'V1_01_easy';

% (2) flag들.
global flag_draw flag_histeq
flag_draw    = true;
flag_histeq  = false;

% (3) 데이터셋 읽어오고, 사용 할 이미지 범위 지정 (data_info를 갈아엎고 싶은데)
global data_info n_start n_final n_cur n_key
n_start = 110;
% n_final = 2890;
n_final = 140;
n_key   = n_start;
n_cur   = n_start + 1;
data_info = loadStereoData(dataset_name, subset_name, dataset_dir, 0);
global objpool_node objpool_elem
% [objpool_node, objpool_elem] = objectpoolBuild(262144); % objectpool을 만든다.

%%
% (4) quadtree 를 저장. 어차피 current image의 left & right tree만 쓴다.
global qts_left qts_right idx_left idx_right node_left node_right
global thres_dist max_depth eps
qts_left  = -1; % 현재 이미지의 왼쪽 트리 주소
qts_right = -1; % 현재 이미지의 오른쪽 트리 주소

idx_left   = []; % left 이미지에 대해 matching 된 점의 번호.
idx_right  = []; % right 이미지에 대해 matching 된 점의 번호.
node_left  = []; % left 이미지에 대해 cached 된 node들
node_right = []; % right 이미지에 대해 cached 된 ndoe들
thres_dist = 30; % 30 px 이상 매칭되면 버린다.
max_depth  = 5; % 최고 깊이.
eps        = 0.0; % epsilon .

n_rows = data_info.intrin.n_rows.left;
n_cols = data_info.intrin.n_cols.left;

% (5) edge 추출 관련
global thres_grad_min thres_grad_max ratio_reduction_right size_sobel overlap len_min len_max
thres_grad_min = 55;
thres_grad_max = 100;
ratio_reduction_right = 0.8; % right 이미지는 최대한 left에서 보이는 모든 점을 포착하기 위해 0.8배 약하게 threshold 설정.
size_sobel = 3;
overlap = 15;
len_min = 10; % 15 pixel 최소 추출.
len_max = 25; % 25 pixel 최대 추출.

% (6) iteration 관련
global MAX_ITER thres_rot thres_trans
MAX_ITER    = 50;
thres_rot   = 3; % degree
thres_trans = 0.4; % [m]

%% 첫 이미지 읽어오기.
% key 스테레오 이미지를 읽어온다.
[img_l, img_r] = imreadStereo(data_info, n_start, flag_histeq);
% key 스테레오 이미지를 undistort & rectify 해준다.
[img_l_rect, img_r_rect, data_info] = stereoRectifyUndist(img_l, img_r, data_info); % 잘 되는걸까...
% key 스테레오 이미지의 gradient & edge & saliency 모두 추출한다.
frame_k = frameConstruct(img_l_rect, img_r_rect, n_start); % 대략 3mb이다.
% key static stereo 복원
frame_k = stereoDisparityStatic(frame_k);
drawValidPoints(frame_k);

figure();imshow([frame_k.left.img,frame_k.right.img],[0,255]);
figure();imshow([frame_k.left.edge(:,:,1)>-1,frame_k.right.edge(:,:,1)>-1],[0,1]);hold on;
plot(frame_k.left.pts(1,:),frame_k.left.pts(2,:),'cx');

K    = data_info.rectify.K;
Kinv = data_info.rectify.Kinv;


figure();
len_temp = 2000;
him = imshow([img_l_rect,img_r_rect],[0,255]);hold on;
hplotl_1 = plot(0,0,'mx','markersize',2);
hplotl_2 = plot(0,0,'bo','markersize',2);
hplotr_1 = plot(0,0,'mx','markersize',2);
hplotr_2 = plot(0,0,'bo','markersize',2);
hline_l = line([zeros(1,len_temp);zeros(1,len_temp)], [zeros(1,len_temp);zeros(1,len_temp)]);
for k = 1:len_temp
   hline_l(k,1).Color = [1,1,1];
   hline_l(k,1).LineWidth = 1.5;
   hline_l(k,1).XData = [0,0];
   hline_l(k,1).YData = [0,0];
end
hline_r = line([zeros(1,len_temp);zeros(1,len_temp)], [zeros(1,len_temp);zeros(1,len_temp)]);
for k = 1:len_temp
   hline_r(k,1).Color = [1,1,1];
   hline_r(k,1).LineWidth = 1.5;
   hline_r(k,1).XData = [0,0];
   hline_r(k,1).YData = [0,0];
end
drawnow;

profile on;
%% 연속 이미지 정렬 및 깊이 복원
xi_ck = zeros(6,1);
g_ck = ones(4,4);
[objpool_node, objpool_elem] = objectpoolBuild(262144); % objectpool을 만든다.

for n_img = n_start + 1:1:n_final
   
   % 현재 이미지 번호.
   n_cur = n_img;
   % 현재 스테레오 이미지를 읽어온다.
   [img_l, img_r] = imreadStereo(data_info, n_cur, flag_histeq);
   % 현재 스테레오 이미지를 undistort & rectify 해준다.
   [img_l_rect, img_r_rect, data_info] = stereoRectifyUndist(img_l, img_r, data_info); % 잘 되는걸까...
   % 현재 스테레오 이미지의 gradient & edge & saliency 모두 추출한다.
   frame_c = frameConstruct(img_l_rect, img_r_rect, n_cur); % 대략 3mb이다.
   % 현재 스테레오에서 quadtree를 생성한다.
   qts_left  = quadtreemBuild(frame_c.left.pts_edge', double(frame_c.left.bins_edge(1,:)'), double(frame_c.left.bins_edge(2,:)'),...
      n_rows, n_cols, max_depth, eps, thres_dist, objpool_node, objpool_elem);
   qts_right = quadtreemBuild(frame_c.right.pts_edge', double(frame_c.right.bins_edge(1,:)'), double(frame_c.right.bins_edge(2,:)'),...
      n_rows, n_cols, max_depth, eps, thres_dist, objpool_node, objpool_elem);
   
   % k의 3d point를 불러옴
   X_k        = frame_k.left.pts3d;
   mask_alive = frame_k.left.is_recon; % 현재 점들의 mask를 담고있다.
   idx_alive  = find(mask_alive);
   
   X_k        = X_k(:,idx_alive);
   dirs1      = double(frame_k.left.dirs(1,idx_alive));
   
   % overlap ratio 계산하고, keyframe 점을 warp함.
   r_prev = 1e15; r_curr = 0;
   
   for iter = 1:MAX_ITER
      
      % 변환을 구한다.
      g_ck = se3Exp(xi_ck);
      g_lr = data_info.extrin.g_nlnr;
      
      % X_k 점들을 워핑한다.
      X_k_warp_l = g_ck(1:3,1:3)*X_k + g_ck(1:3,4);
      X_k_warp_r = g_lr(1:3,1:3)^-1*(X_k_warp_l - g_lr(1:3,4));
      % X_k 점들을 프로젝션한다.      
      pts_k_warp_l = project3to2(X_k_warp_l, K);
      pts_k_warp_r = project3to2(X_k_warp_r, K); % 여기서 에러난다!. 가끔 음수좌표가 들어온다.
      
      % 매칭: iteration 이 1보다 크면 Cached 로 한다.
      if(iter > 1)
         [idx_left, node_left] = quadtreemCachedNN(qts_left, pts_k_warp_l', dirs1',node_left);
         [idx_right, node_right] = quadtreemCachedNN(qts_right, pts_k_warp_r', dirs1',node_right);
      else
         [idx_left, node_left] = quadtreemNN(qts_left, pts_k_warp_l', dirs1');
         [idx_right, node_right] = quadtreemNN(qts_right, pts_k_warp_r', dirs1');
      end
      
      % mask update
      mask_matched = and( (idx_left > 0 ), (idx_right > 0));
      idx_matched  = find(mask_matched); % 현재 살아있는 점들.
      
      idx_left     = idx_left(idx_matched);
      idx_right    = idx_right(idx_matched);
      pts_k_warp_l = pts_k_warp_l(:,idx_matched);
      pts_k_warp_r = pts_k_warp_r(:,idx_matched);
      
      n_pts_k = length(idx_matched); % 유효한 k의 점들.
      Jt = zeros(6, 2*n_pts_k); % left - right 때문에 2배다.
      rt = zeros(1, 2*n_pts_k);
      
      % Jacobian과 residual 을 계산한다.
      % (1) 매칭된 점의 좌표와 gradient를 꺼낸다.
      grad_c_l = [frame_c.left.grad_edge(1,idx_left)./frame_c.left.grad_edge(3,idx_left);...
         frame_c.left.grad_edge(2,idx_left)./frame_c.left.grad_edge(3,idx_left)];
      grad_c_r = [frame_c.right.grad_edge(1,idx_right)./frame_c.right.grad_edge(3,idx_right);...
         frame_c.right.grad_edge(2,idx_right)./frame_c.right.grad_edge(3,idx_right)];
      pts_c_l  = frame_c.left.pts_edge(:,idx_left);
      pts_c_r  = frame_c.right.pts_edge(:,idx_right);
      
      % (2) residual 계산
      rt(1,1:n_pts_k) = grad_c_l(1,:).*(pts_k_warp_l(1,:) - pts_c_l(1,:)) + grad_c_l(2,:).*(pts_k_warp_l(2,:) - pts_c_l(2,:));
      rt(1,n_pts_k+1:2*n_pts_k) = grad_c_r(1,:).*(pts_k_warp_r(1,:) - pts_c_r(1,:)) + grad_c_r(2,:).*(pts_k_warp_r(2,:) - pts_c_r(2,:));
      % (3) Jacobian 계산
      gfu = grad_c_l(1,:)*K(1,1);
      gfv = grad_c_l(2,:)*K(2,2);
      iz  = 1./X_k_warp_l(3,idx_matched);
      xiz = X_k_warp_l(1,idx_matched).*iz;
      yiz = X_k_warp_l(2,idx_matched).*iz;
      Jt(1,1:n_pts_k) = gfu.*iz;
      Jt(2,1:n_pts_k) = gfv.*iz;
      Jt(3,1:n_pts_k) = (-gfu.*xiz - gfv.*yiz).*iz;
      Jt(4,1:n_pts_k) = -gfu.*xiz.*yiz - gfv.*(1 + yiz.*yiz);
      Jt(5,1:n_pts_k) = gfu.*(1 + xiz.*xiz) + gfv.*xiz.*yiz;
      Jt(6,1:n_pts_k) = -gfu.*yiz + gfv.*xiz;
      
      gfu = grad_c_r(1,:)*K(1,1);
      gfv = grad_c_r(2,:)*K(2,2);
      iz  = 1./X_k_warp_r(3,idx_matched);
      xiz = X_k_warp_r(1,idx_matched).*iz;
      yiz = X_k_warp_r(2,idx_matched).*iz;
      Jt(1,n_pts_k+1:2*n_pts_k) = gfu.*iz;
      Jt(2,n_pts_k+1:2*n_pts_k) = gfv.*iz;
      Jt(3,n_pts_k+1:2*n_pts_k) = (-gfu.*xiz - gfv.*yiz).*iz;
      Jt(4,n_pts_k+1:2*n_pts_k) = -gfu.*xiz.*yiz - gfv.*(1 + yiz.*yiz);
      Jt(5,n_pts_k+1:2*n_pts_k) = gfu.*(1 + xiz.*xiz) + gfv.*xiz.*yiz;
      Jt(6,n_pts_k+1:2*n_pts_k) = -gfu.*yiz + gfv.*xiz;
      
      % Weight 계산은 필수다. 잘못 매칭된 애들을 걸러내기 위함.
      thres_huber = 2.5;
      index_huber = find(abs(rt)>thres_huber);
      W = ones(1,2*n_pts_k);
      W(1,index_huber) = thres_huber./abs(rt(index_huber));
      
      % (4) Hessian 계산
      JtW = bsxfun(@times,Jt,W);
      H = JtW*Jt';
      % (5) delta_xi 계산
      delta_xi = -H^-1*JtW*rt';
      % (6) update
      xi_ck = xi_ck + delta_xi;
      
      r_norm = sqrt(sum(rt.^2)/length(rt));
      fprintf('ncur: %d / iter: %d / norm: %0.6f\n', n_img, iter, r_norm);
      fprintf('angle: %0.3f\n', 180/pi*norm(r2a(g_ck(1:3,1:3))));
   end
   ratio_overlap = calcOverlapRatio(frame_k, g_ck);
   fprintf('overlap: %0.3f\n',ratio_overlap);
   
   % 깊이 update.
   % frame_c 를 이용해서 static update 시행. (단, 2 cm 이상 이동 & 0.5도 이상회전)
   if(norm(g_ck(1:3,4)) > 0.03 && norm( r2a(g_ck(1:3,1:3)) ) >= 0.1 /180*pi)
      % warp 후, frame_c에서 static stereo 시행.
      frame_k = stereoDisparityStaticWarped(frame_c, frame_k, g_ck);
      
      % static stereo 결과를 frame_k로 옮겨줌.
      frame_k = stereoDisparityTemporal(frame_k, frame_c, g_ck);
      
      drawValidPoints(frame_k);
      % frame_c 를 이용해서 temporal update 시행. (단 2 cm 이상 이동 시)
      fprintf('another static on: %d\n',n_img);
   end
   
   %    % update keyframe to , center의 각도가 30 도 이상 벌어지면 업데이트.
   %    if(ratio_overlap <= 0.7 & norm(g_k(1:3,4)-g_c(1:3,4))>0.15)
   %       X_c    = frame_c.left.pts3d(:,find(frame_c.left.is_recon));
   %       X_c = data_info.gt.g(1:3,1:3,n_img)*X_c + data_info.gt.g(1:3,4,n_img);
   %       plot3(X_c(1,:),X_c(2,:),X_c(3,:),'r.','markersize',1);
   %       drawnow;
   %       frame_k = frame_c;
   %
   %    end
   
   
   % keyframe update 조건 결정.
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
%    objectpoolDelete(objpool_node, objpool_elem);
end

profile viewer;