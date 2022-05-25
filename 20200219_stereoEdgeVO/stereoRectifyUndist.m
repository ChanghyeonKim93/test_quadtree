function [img_l_rect,img_r_rect,data_info] = stereoRectifyUndist(img_l, img_r, data_info)
persistent p_l p_r K_rect R_cl R_cr p_l_distorted p_r_distorted
% persistent n_cols n_rows
% global data_info
K_l = data_info.intrin.K.left;
K_r = data_info.intrin.K.right;
distortion_l = data_info.intrin.distortion.left;
distortion_r = data_info.intrin.distortion.right;

flag_distortion_mode = 5;
n_cols = data_info.intrin.n_cols.left;
n_rows = data_info.intrin.n_rows.left;
if(isempty(p_l_distorted) == true)
   g_lr = data_info.extrin.g_clcr;
   
   g_0l = eye(4,4);
   g_0r = g_lr;
   
   R_0l = g_0l(1:3,1:3);
   R_0r = g_0r(1:3,1:3);
   
   k_l = R_0l(:,3); % 이게 맞다
   k_r = R_0r(:,3);
   k_n = (k_l + k_r)*0.5;
   k_n = k_n/norm(k_n);
   
   i_n = g_lr(1:3,4);
   i_n = i_n/norm(i_n);
   
   j_n = cross(k_n, i_n);
   j_n = j_n/norm(j_n);
   
   R_0n = [i_n,j_n,k_n];
   % New intrinsic parameter
   f_n   = (K_l(1,1) + K_r(1,1))/1.95;
   
   centu = n_cols/2;
   centv = n_rows/2;
   K_rect =...
      [f_n,0,centu;...
      0,f_n,centv;...
      0,0,1];
   
   [X,Y] = meshgrid(1:n_cols,1:n_rows);
   
   u_vec = X(:).';
   v_vec = Y(:).';
   p_n = [u_vec; v_vec; ones(1,length(u_vec))];
   P_0 = R_0n*K_rect^-1*p_n;
   
   P_l = R_0l^-1*P_0;
   P_l = P_l./P_l(3,:);
   
   P_r = R_0r^-1*P_0;
   P_r = P_r./P_r(3,:);
   
   p_l = K_l*P_l;
   p_r = K_r*P_r;
   
   if(flag_distortion_mode == 2)
      % Project to distorted pixel coordinate
      x = (p_l(1,:)-K_l(1,3))/K_l(1,1);
      y = (p_l(2,:)-K_l(2,3))/K_l(2,2);
      r = sqrt(x.^2 + y.^2);
      r_radial = ones(size(r)) + distortion_l(1)*r.^2 + distortion_l(2)*r.^4;
      p_l_distorted = [K_l(1,3) + K_l(1,1)*x.*r_radial;K_l(2,3) + K_l(2,2)*y.*r_radial];
      
      x = (p_r(1,:)-K_r(1,3))/K_r(1,1);
      y = (p_r(2,:)-K_r(2,3))/K_r(2,2);
      r = sqrt(x.^2 + y.^2);
      r_radial = ones(size(r)) + distortion_r(1)*r.^2 + distortion_r(2)*r.^4;
      p_r_distorted = [K_r(1,3) + K_r(1,1)*x.*r_radial;K_r(2,3) + K_r(2,2)*y.*r_radial];
      
   elseif(flag_distortion_mode == 5)
      k1 = distortion_l(1);
      k2 = distortion_l(2);
      k3 = distortion_l(5);
      p1 = distortion_l(3);
      p2 = distortion_l(4);
      
      x = (p_l(1,:)-K_l(1,3))/K_l(1,1);
      y = (p_l(2,:)-K_l(2,3))/K_l(2,2);
      r = sqrt(x.^2 + y.^2);
      r_radial = ones(size(r)) + k1*r.^2 + k2*r.^4 + k3*r.^6;
      x_dist = x.*r_radial + 2*p1*x.*y + p2*(r.^2 + 2*x.^2);
      y_dist = y.*r_radial + p1*(r.^2 + 2*y.^2) + 2*p2*x.*y;
      p_l_distorted = [K_l(1,3) + x_dist*K_l(1,1);K_l(2,3) + y_dist*K_l(2,2)];
      
      
      k1 = distortion_r(1);
      k2 = distortion_r(2);
      k3 = distortion_r(5);
      p1 = distortion_r(3);
      p2 = distortion_r(4);
      x = (p_r(1,:)-K_r(1,3))/K_r(1,1);
      y = (p_r(2,:)-K_r(2,3))/K_r(2,2);
      r = sqrt(x.^2 + y.^2);
      r_radial = ones(size(r)) + k1*r.^2 + k2*r.^4 + k3*r.^6;
      x_dist = x.*r_radial + 2*p1*x.*y + p2*(r.^2 + 2*x.^2);
      y_dist = y.*r_radial + p1*(r.^2 + 2*y.^2) + 2*p2*x.*y;
      p_r_distorted = [K_r(1,3) + x_dist*K_r(1,1);K_r(2,3) + y_dist*K_r(2,2)];
   else
      
   end
   
   R_ln = R_0l^-1*R_0n;
   R_rn = R_0r^-1*R_0n;
   K_n = K_rect;
   
   data_info.rectify.K = K_n;
   data_info.rectify.Kinv = K_n^-1;
   data_info.extrin.g_nlnr = [eye(3,3), R_ln^-1*g_lr(1:3,4); 0,0,0,1];
end

img_l_rect = reshape(interpImage(img_l,[p_l_distorted(1,:);p_l_distorted(2,:)]),n_rows,n_cols);
img_r_rect = reshape(interpImage(img_r,[p_r_distorted(1,:);p_r_distorted(2,:)]),n_rows,n_cols);
img_l_rect = round(img_l_rect);
img_r_rect = round(img_r_rect);

end

