function frame_k = updateInverseDepth(frame_k, invd_save, std_invd_save, flag_valid)
global data_info
K    = data_info.rectify.K; % rectification 된 K
Kinv = data_info.rectify.Kinv;% rectification 된 Kinv

flag_valid_prev = frame_k.left.is_recon;
n_pts = sum(flag_valid_prev);

% invd_save 길이와 frame_k.left.pts 길이가 같아야 한다.
for k = 1:n_pts
   if(flag_valid(k) > 0)
      invd_prev = frame_k.left.invd(k);
      std_prev  = frame_k.left.std_invd(k);
      
      invd_curr = invd_save(k);
      std_curr  = std_invd_save(k);
      if(invd_prev > 0) % (1) inverse depth가 있던 경우.
         % 퓨전.
         invd_fusion = 1/(std_prev^2+std_curr^2)*(invd_prev*std_curr^2 + invd_curr*std_prev^2);
         std_fusion   = sqrt( (std_prev^2*std_curr^2)/(std_prev^2+std_curr^2) );
         
         % inverse depth를 계산한다.
         frame_k.left.invd(k)     = invd_fusion;
         % standard deviation을 계산한다.
         frame_k.left.std_invd(k) = std_fusion;
         % pts3d를 계산한다.
         frame_k.left.pts3d(:,k) = Kinv*[frame_k.left.pts(:,k);1]/invd_fusion;
      else
         % inverse depth를 계산한다.
         frame_k.left.invd(k)     = invd_curr;
         % standard deviation을 계산한다.
         frame_k.left.std_invd(k) = std_curr;
         % pts3d를 계산한다.
         frame_k.left.pts3d(:,k)  = Kinv*[frame_k.left.pts(:,k);1]/invd_curr;
      end
   end
end
end