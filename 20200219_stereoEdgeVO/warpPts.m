function pts_warp = warpPts(pts, depth, g_temp)
global data_info
K    = data_info.rectify.K;
Kinv = data_info.rectify.Kinv;

X_temp = g_temp(1:3,1:3)*Kinv*[pts(1,:); pts(2,:); ones(1,size(pts,2))].*depth + g_temp(1:3,4);
pts_temp = K*(X_temp./X_temp(3,:));
pts_warp = pts_temp(1:2,:);
end