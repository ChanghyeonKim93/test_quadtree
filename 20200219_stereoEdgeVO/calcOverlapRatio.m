function ratio = calcOverlapRatio(frame_k, g_ck)
global data_info
n_cols = data_info.intrin.n_cols.left;
n_rows = data_info.intrin.n_rows.left;
K = data_info.rectify.K;

mask_recon = frame_k.left.is_recon;
idx_recon = find(mask_recon);
X_k = frame_k.left.pts3d(:, idx_recon);
X_k_warp_l = g_ck(1:3,1:3)*X_k + g_ck(1:3,4);
pts_k_warp_l = [K(1,1)*X_k_warp_l(1,:)./X_k_warp_l(3,:) + K(1,3);...
   K(2,2)*X_k_warp_l(2,:)./X_k_warp_l(3,:) + K(2,3)];

mask_overlap = (pts_k_warp_l(1,:) < n_cols) &  (pts_k_warp_l(1,:) > 0)...
   & (pts_k_warp_l(2,:) < n_rows) &  (pts_k_warp_l(2,:) > 0);

ratio = sum(mask_overlap)/length(mask_overlap);
end