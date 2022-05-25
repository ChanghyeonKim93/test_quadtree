function points = triangulatePointsDLT(pts1,pts2,g_21)
global data_info
K = data_info.rectify.K;

len    = size(pts1,2);
points = zeros(3,len);

R_21 = g_21(1:3,1:3);
T_21 = g_21(1:3,4);

% P1 = K*[eye(3,3),zeros(3,1)];
P1 = [K,zeros(3,1)];
P2 = K*[R_21,T_21];

for i = 1:len
   A = [pts1(2,i)*P1(3,:)-P1(2,:);...
      P1(1,:)-pts1(1,i)*P1(3,:);...
      pts2(2,i)*P2(3,:)-P2(2,:);...
      P2(1,:)-pts2(1,i)*P2(3,:)];
   [~,~,V] = svd(A); % minimum singular value.
   X_temp = V(:,4);
   points(:,i) = X_temp(1:3)./X_temp(4);
end

end