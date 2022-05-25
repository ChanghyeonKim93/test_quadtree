function drawValidPoints(frame)
% close all;
X = frame.left.pts3d(:,find(frame.left.is_recon));
std_invd = frame.left.std_invd(1,find(frame.left.is_recon));
n_pts = length(X);

% std에 따라서 색상 정하기. std 가 클수록 빨간색,
colors = zeros(n_pts,3);
marker_areas = 4*ones(1,n_pts);

std_worst = 0.06;
std_best  = 0.01;
slope = 1/(std_best-std_worst);

% hue 0.3이 green에 해당. hue 0 red hue 1 red. 순환하니까!
for i = 1:n_pts
   hue_temp = slope*(std_invd(i) - std_worst);
   if(hue_temp >= 0.3)
      hue_temp = 0.3;
   elseif(hue_temp<=0)
      hue_temp = 0;
   end
   colors(i,:) = hsv2rgb([hue_temp,1,1]);
end

figure(66); 
hold off;
scatter3(X(1,:),X(2,:),X(3,:),marker_areas,colors,'filled','MarkerFaceAlpha',0.6);hold on; grid minor; axis equal;
view(90,0);
set(gcf,'Color','k');
set(gca,'Color','k');
set(gca,'XColor','white');
set(gca,'YColor','white');
set(gca,'ZColor','white');
drawnow;

end