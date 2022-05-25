function frame = frameConstruct(img_l_rect, img_r_rect, n_img)
global size_sobel thres_grad_min thres_grad_max ratio_reduction_right
global overlap len_min len_max

% stereo Frame 하나를 만든다.
% frame  << 어떠한 frame에 관한 전반적인 모든 것
%    L id           : 1 x 1  현재 camera frame의 고유 번호. 
%    L left (right) : struct 현재 camera frame에서 관측된 left (right) 이미지에 관한 모든 정보들.
%        L img      : H x W  현재 이미지 (rectified)
%        L du       : H x W  gradient - u
%        L dv       : H x W  gradient - v
%        L dmag     : H x W  gradient magnitude
%        L edge     : H x W x 2 각 엣지의 방향 이미지. -1이 아니면 엣지가 있는거다.

%       edge 점들과 관련된 것.
%        L pts_edge : 1 x N 전체 엣지 점들.
%        L bins_edge: 2 x N 전체 엣지 점들의 방향.
%        L grad_edge: 3 x N 전체 엣지 점들의 gradient & magnitude (3행)

%       2차원 점 중 복원 대상과 관련된 것.
%        L pts      : 2 x n 선택받은 점들. (center 점들)
%        L dirs     : 1 x n 선택받은 점들의 주된 방향.
%        L grad     : 3 x n 선택받은 점들의 gradient & magnitude

%       3차원 점과 관련된 것.
%        L pts3d    : 3 x n 선택받은 점들의 3차원 복원
%        L invd     : 1 x n inverse depth
%        L d        : 1 x n depth
%        L std_invd : 1 x n 표준편차 of inverse depth
%        L std_d    : 1 x n 표준편차 of depth
%        L is_recon : 1 x n 깊이가 복원 되었는지 여부.
%        L df : depth filter 관련.
%          L a        : 1 x n
%          L b        : 1 x n
%          L mu       : 1 x n
%          L sig      : 1 x n
%          L z_min    : 1 x n
%          L z_max    : 1 x n

%       매칭과 관련된 것. (dirs 이용함)
%        L match_idx  : 1 x n 점들이 tree에 매칭 된 위치.
%        L match_node : 1 x n 점들이 tree에서 매칭 된 이전 node 위치.

% 스테레오 이미지의 gradient & edge 를 추출한다.
sobel_num=1;
if(size_sobel == 3)
   sobel_num = 1;
elseif(size_sobel == 5)
   sobel_num = 4.5;
end
du_l   = cv.Sobel(img_l_rect,'KSize',size_sobel,'XOrder',1,'YOrder',0); % 5x5: 18, 3x3: 4
dv_l   = cv.Sobel(img_l_rect,'KSize',size_sobel,'XOrder',0,'YOrder',1);
dmag_l = abs(du_l)+abs(dv_l);
edge_l = double(cv.Canny2(du_l, dv_l, [thres_grad_min, thres_grad_max]*sobel_num));
du_r   = cv.Sobel(img_r_rect,'KSize',size_sobel,'XOrder',1,'YOrder',0); % 5x5: 18, 3x3: 4
dv_r   = cv.Sobel(img_r_rect,'KSize',size_sobel,'XOrder',0,'YOrder',1);
dmag_r = abs(du_r)+abs(dv_r);
edge_r = double(cv.Canny2(du_r, dv_r, [thres_grad_min, thres_grad_max]*sobel_num*ratio_reduction_right));

% 추출한 edge의 saliency를 체크한다.
[pts_edge_l, bins_l, grad_l, dir1_img_l, dir2_img_l, edgelets_l] =...
   findSalientEdges(edge_l, du_l, dv_l, overlap, len_min, len_max); % gradient도 주자. dir image도 필요하네....
dirs_img_l = zeros(size(img_l_rect,1),size(img_l_rect,2),2);
dirs_img_l(:,:,1)=dir1_img_l;
dirs_img_l(:,:,2)=dir2_img_l;

[pts_edge_r, bins_r, grad_r, dir1_img_r, dir2_img_r, edgelets_r] =...
   findSalientEdges(edge_r, du_r, dv_r, overlap, len_min, len_max); % gradient도 주자.
dirs_img_r = zeros(size(img_r_rect,1),size(img_r_rect,2),2);
dirs_img_r(:,:,1)=dir1_img_r;
dirs_img_r(:,:,2)=dir2_img_r;

% Warping 할 left 점들이다.
pts_l  = edgelets_l.pts_centers;
dirs_l = edgelets_l.bins_centers;
grad_centers_l = zeros(3,length(pts_l));
grad_centers_l(1,:) = interpImage(du_l,pts_l)';
grad_centers_l(2,:) = interpImage(dv_l,pts_l)';
grad_centers_l(3,:) = abs(grad_centers_l(1,:)) + abs(grad_centers_l(2,:));

n_pts_l = length(pts_l);
df_left= struct('a',0.5*ones(1,n_pts_l),'b',0.5*ones(1,n_pts_l),'mu',zeros(1,n_pts_l),'sig',5*ones(1,n_pts_l),'z_min',zeros(1,n_pts_l),'z_max',40*ones(1,n_pts_l));
left  = struct('img',img_l_rect, 'du',du_l, 'dv',dv_l, 'dmag',dmag_l,'edge',int8(dirs_img_l),...
   'pts_edge',pts_edge_l,'bins_edge',int8(bins_l),'grad_edge',grad_l,...
   'pts',pts_l,'dirs',dirs_l,'grad',grad_centers_l,...
   'pts3d',zeros(3,n_pts_l),'invd',zeros(1,n_pts_l),'d',zeros(1,n_pts_l),'std_invd',zeros(1,n_pts_l),'std_d',zeros(1,n_pts_l),'is_recon',int8(zeros(1,n_pts_l)),'df',df_left,...
   'match_idx',zeros(1,n_pts_l),'match_node',zeros(1,n_pts_l));
right = struct('img',img_r_rect, 'du',du_r, 'dv',dv_r, 'dmag',dmag_r,'edge',int8(dirs_img_r),...
   'pts_edge',pts_edge_r,'bins_edge',int8(bins_r),'grad_edge',grad_r,...
   'pts',[],'dirs',[],'grad',[],...
   'pts3d',[],'invd',[],'d',[],'std_invd',[],'std_d',[],'is_recon',[],'df',[],...
   'match_idx',[],'match_node',[]); % right는 어차피 깊이 값 필요없다 .

frame = struct('id',n_img,'left',left,'right',right);

% figure(); imshow(edge_l,[0,255]); hold on;
% plot(pts_l(1,:), pts_l(2,:),'yo','markersize',4,'linewidth',2); title('generated first image');
end