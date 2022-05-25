function frame_k = stereoDisparityTemporal(frame_k, frame_c, g_ck)
% �� �Լ����� ������Ʈ �ؾ��ϴ� ��.
% left �̹������� ���ؼ� temporal update �� �����Ѵ�.

% intrinsic �� extrinsic load
global data_info
n_cols = data_info.intrin.n_cols.left;
n_rows = data_info.intrin.n_rows.left;
K    = data_info.rectify.K; % rectification �� K
Kinv = data_info.rectify.Kinv;% rectification �� Kinv

R_ck = g_ck(1:3,1:3);
t_ck = g_ck(1:3,4);
t_kc = -R_ck'*t_ck;

% Fundamental ��� & Essential ���
F_ck = Kinv'*hat(t_ck)*R_ck*Kinv;

% key & cur �̹��� �ε�
img_k = frame_k.left.img;
img_c = frame_c.left.img;

% ���� ��� �� & �� ������ �׷����Ʈ
pts_k  = frame_k.left.pts;
grad_k = frame_k.left.grad;

n_pts       = length(pts_k);
scores_best = -ones(1,n_pts);
flag_valid  = zeros(1,n_pts);

invd_save   = -ones(1,n_pts);
std_invd_save = -ones(1,n_pts);

% �Ķ���� ����
win_sz      = 8; % half length of window.
fat_sz      = 3; % half thicKess of window.

angle_thres  = 60; % �� �������� grad �� epiline�� ������ ũ�� ���� x
cosd_thres   = cosd(angle_thres);
thres_zncc1  = 0.95; % NCC�� �� ���� ���� ������ ���� X
thres_zncc2  = 0.75; % NCC�� �� ���� �Ѵ� �༮�� �������� ������ üũ�غ���.
thres_multipeak = 4;

eps_edge     = 0.5; % ���� ���� std: 0.5 [px]
eps_epi      = 1;   % epipolar line ���� std: 1 [px]

% minimum & maximum disparity ��. �ش� disparity ���� �ȿ����� ��Ī.
% baseline ���, 0.4 m ~ 15 m �������� ������ disparity ���� ������ ����صд�.
baseline  = norm(g_ck(1:2,4)); % [m] Z �������� ���°� ����?
focal     = K(1,1);

bf = baseline*focal;
bfinv = 1/bf;

d_min_default = 0.1; % 0.1 [m] % 0.1m ����,
d_max_default = 40;  % 30  [m] % 40m ���� ������.

fprintf('max disp: %0.0f / min disp: %0.0f\n', bf/d_min_default, bf/d_max_default);

% ��ġ ���ø�!
[X,Y] = meshgrid(-win_sz:win_sz, -fat_sz:fat_sz);
pts_patch_template = [X(:)';Y(:)'];

%% �������� ���� ����
for i = 1:n_pts
   % ���� i ��° ���� �����͸� �����Ѵ�.
   pt_k = pts_k(:,i);
   du_k = grad_k(1,i);
   dv_k = grad_k(2,i);
   dmag_k = sqrt(du_k^2+dv_k^2);
   
   du_k = du_k / dmag_k;
   dv_k = dv_k / dmag_k;
   
   % cur �� ���� epipolar line ������ vector�� u������ ã�´�. l_c,l0_c;
   coefficient_c = F_ck*[pt_k;1];
   l_c  = [coefficient_c(2),-coefficient_c(1)]';
   l_c  = l_c/norm(l_c); % normalized vector.
   pt_c_arbi = warpPts(pt_k, 10, g_ck);
   % key �� ���� epipolar line ������ vector�� uã�´�. l_k, l0_k;
   coefficient_k = [pt_c_arbi;1]'*F_ck;
   l_k  = [coefficient_k(2),-coefficient_k(1)]';
   l_k  = l_k/norm(l_k);
   l0_k = [-coefficient_k(3)/coefficient_k(1),0]';
   
   % �� ������ ������ �����ش�. (������ ����̸� ��)
   if(dot(l_c,l_k) < 0)
      l_c = -l_c;
   end
   
   cos_th = dot(l_k,[du_k,dv_k]);
   sin_th = sqrt(1-cos_th^2);
   
   % gradient ������ key epi line�� �̷�� ������ 60�� �̳��� ����.
   if( (abs(cos_th) >= cosd_thres) && ...
         (pt_k(1) > win_sz + 1) && (pt_k(1) < n_cols - win_sz - 1) &&...
         (pt_k(2) > win_sz + 1) && (pt_k(2) < n_rows - win_sz - 1))
      
      % �ϴ� valid �����.
      flag_valid(i) = 1;
      % ��ġ�� �����.
      pts_k_patch = calcEpiPatchTemplate(pt_k, l_k, pts_patch_template);
      patch_k     = interpImage(img_k, pts_k_patch);
      
      % ����, ���� ���� �̹� �ִٸ�, (������ ���ε�) +-2*std �������� disparity ��Ī ���� ����
      if(frame_k.left.invd(i) > 0)% ���̰��� ���ٸ�, �ִ� range����
         invd_temp = frame_k.left.invd(i);
         std_temp  = frame_k.left.std_invd(i);
         d_min = 1/(invd_temp + 2*std_temp);
         d_max = 1/(invd_temp - 2*std_temp);
      else
         d_min = d_min_default;
         d_max = d_max_default;
      end
      % cur �̹������ٴ� ������ Ȯ�����ش�.
      if(d_min < t_kc(3) + d_min_default)
         d_min = t_kc(3) + d_min_default;
      end
      % ������ ���� �ۿ� �ִ� ���̶�� �Ⱒ
      if(d_max > d_max_default)
         flag_valid(i) = 0;
      end
      % ���� �˰��ִ� depth�� min max ���� �̿��ؼ� keyframe���� current frame���� �������ش�.
      pt_c_start = warpPts(pt_k, d_max, g_ck);
      pt_c_end   = warpPts(pt_k, d_min, g_ck);
      
      % �ٿ������ pt_end - pt_start ������ ��ġ���� Ȯ���غ���.
      % ��Ī������ �̹��� ���� / �ִ� & �ּ� ���� ������ ����Ͽ� ������ �����ش�.
%       [pt_c_start, pt_c_end, n_search] = testBoundary(pt_c_start, pt_c_end, l_c, n_rows, n_cols);
      
      if( dot(l_c,pt_c_end-pt_c_start) < 0 ) % l_c�� ���� ���� �����.
         pt_temp = pt_c_start;
         pt_c_start = pt_c_end;
         pt_c_end = pt_temp;
      end
      n_search = round(norm(pt_c_end - pt_c_start));
      
      % ��Ī���� ã�´�
      scores       = -ones(1,n_search); % ��Ī�ϴ� ���� ��� score����.
      score_best   = -1e5;
      pt_c_over = []; % thres2�� (0.8)�� ���� ������ ��Ƶ�.
      pt_c_best = zeros(2,1); % ��Ī�� ���� ��ġ. (subpixel)
      
      % ������ ��ȸ�ϸ� ��Ī.
      ind_best = -1;
      for j = 1:n_search
         % ���� ��ǥ
         pt_c_now =  j*l_c + pt_c_start;
         % ��ġ�� ���Ѵ�.
         pts_c_patch = calcEpiPatchTemplate(pt_c_now, l_c, pts_patch_template);
         patch_c     = interpImage(img_c, pts_c_patch);         
         % ���ھ ���Ѵ�.
         scores(j)   = calcZNCC(patch_k(:), patch_c(:));
         % ���ھ ���Ѵ�.
         if(scores(j) > thres_zncc2)
            pt_c_over = [pt_c_over, pt_c_now];
         end
         % ����, ���� ZNCC�� ���� �� ���� ������ �ٲ۴�.
         if(scores(j) > score_best)
            score_best = scores(j);
            pt_c_best = pt_c_now;
            ind_best = j;
         end
      end
      
      % �ְ� ������ threshold�� �Ѱ�, ��� u_over�� 6 pixel �̳��̰�,
      % ���� �����ڸ��� �ƴ� ��쿡�� �ŷ�.
      if(n_search < 4)
         flag_valid(i)=0;
      end
      if(ind_best < 2 || ind_best > n_search-2)
         flag_valid(i)=0;
      end
      if(flag_valid(i))
         for j = 1:size(pt_c_over,2) % �ϳ��� 6 px �̻� ����ִٸ� �Ⱒ.
            if(norm(pt_c_best - pt_c_over(:,j)) > thres_multipeak)
               flag_valid(i) = 0;
               break;
            end
         end
      end
      
      if(flag_valid(i) && (score_best > thres_zncc1))
         % �ְ� ������ �����Ѵ�.
         scores_best(i) = score_best;
         % score history �̿��ؼ� 2���Լ� �𵨷� subpixel refinement ����.
         s1 = scores(ind_best-1); % ���� ��.
         s2 = scores(ind_best); % �߽� ��.
         s3 = scores(ind_best+1); % ���� ��.
         pt_c_best = pt_c_best - (s3-s1)/(s3+s1-2*s2)*0.5*l_c; % refine�� ��ǥ ��.
         
         X = triangulatePointsDLT(pt_k,pt_c_best,g_ck);
         
         % inverse depth�� ����Ѵ�.
         invd_save(i) = 1/X(3);
         
         % standard deviation�� ����Ѵ�.
         std_invd_save(i) = 1/abs(cos_th)*sqrt(eps_edge^2 + eps_epi^2*(1-cos_th^2))*bfinv;
      else
         flag_valid(i) = 0;
      end
   end
end

% % ��� ������Ʈ
for k = 1:n_pts
   if(flag_valid(k) > 0)
      invd_prev = frame_k.left.invd(k);
      std_prev = frame_k.left.std_invd(k);

      invd_curr = invd_save(k);
      std_curr  = std_invd_save(k);
      if(invd_prev > 0) % (1) inverse depth�� �ִ� ���.
         % ǻ��.
         invd_fusion = 1/(std_prev^2+std_curr^2)*(invd_prev*std_curr^2 + invd_curr*std_prev^2);
         std_fusion   = sqrt( (std_prev^2*std_curr^2)/(std_prev^2+std_curr^2) );

         % inverse depth�� ����Ѵ�.
         frame_k.left.invd(k)     = invd_fusion;
         % standard deviation�� ����Ѵ�.
         frame_k.left.std_invd(k) = std_fusion;
         % pts3d�� ����Ѵ�.
         frame_k.left.pts3d(:,k) = Kinv*[frame_k.left.pts(:,k);1]/invd_fusion;
         frame_k.left.is_recon(k) = 1;
      else
         % inverse depth�� ����Ѵ�.
         frame_k.left.invd(k)     = invd_curr;
         % standard deviation�� ����Ѵ�.
         frame_k.left.std_invd(k) = std_curr;
         % pts3d�� ����Ѵ�.
         frame_k.left.pts3d(:,k)  = Kinv*[frame_k.left.pts(:,k);1]/invd_curr;
         frame_k.left.is_recon(k) = 1;
      end
   end
end

% ��� ������Ʈ
% for k = 1:n_pts
%    if(flag_valid(k) > 0)
%       invd_prev = frame_k.left.invd(k);
%       std_prev = frame_k.left.std_invd(k);
%       
%       invd_curr = invd_save(k);
%       std_curr  = std_invd_save(k);
%       if(invd_prev > 0) % (1) inverse depth�� �ִ� ���.
%          % ǻ��.
%          a_prev   = frame_k.left.df.a(k);
%          b_prev   = frame_k.left.df.b(k);
%          mu_prev  = frame_k.left.df.mu(k);
%          sig_prev = frame_k.left.df.sig(k);
%          z_min_prev = frame_k.left.df.z_min(k);
%          z_max_prev = frame_k.left.df.z_max(k);
%          [a_new, b_new, mu_new, sig_new, z_min_new, z_max_new]...
%             =updateDF(invd_curr, std_curr,a_prev,b_prev, mu_prev, sig_prev, z_min_prev, z_max_prev);
%          frame_k.left.df.a(k) = a_new;
%          frame_k.left.df.b(k) = b_new;
%          frame_k.left.df.mu(k) = mu_new;
%          frame_k.left.df.sig(k) = sig_new;
%          frame_k.left.df.z_min(k) = z_min_new;
%          frame_k.left.df.z_max(k) = z_max_new;
%          
%          %          invd_fusion = 1/(std_prev^2+std_curr^2)*(invd_prev*std_curr^2 + invd_curr*std_prev^2);
%          %          std_fusion   = sqrt( (std_prev^2*std_curr^2)/(std_prev^2+std_curr^2) );
%          
%          % inverse depth�� ����Ѵ�.
%          frame_k.left.invd(k)     = mu_new;
%          % standard deviation�� ����Ѵ�.
%          frame_k.left.std_invd(k) = sig_new;
%          % pts3d�� ����Ѵ�.
%          frame_k.left.pts3d(:,k) = Kinv*[frame_k.left.pts(:,k);1]/mu_new;
%          frame_k.left.is_recon(k) = 1;
%          
%       else % �ƹ��͵� ���� ���.
%          frame_k.left.df.mu(k) = invd_curr;
%          frame_k.left.df.sig(k) = std_curr;
%          if(frame_k.left.df.z_min(k) > invd_curr)
%             frame_k.left.df.z_min(k) = invd_curr;
%          end
%          if(frame_k.left.df.z_max(k) < invd_curr)
%             frame_k.left.df.z_max(k) = invd_curr;
%          end
%          % inverse depth�� ����Ѵ�.
%          frame_k.left.invd(k)     = invd_curr;
%          % standard deviation�� ����Ѵ�.
%          frame_k.left.std_invd(k) = std_curr;
%          % pts3d�� ����Ѵ�.
%          frame_k.left.pts3d(:,k)  = Kinv*[frame_k.left.pts(:,k);1]/invd_curr;
%          frame_k.left.is_recon(k) = 1;
%       end
%    end
% end
end