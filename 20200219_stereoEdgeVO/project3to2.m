function pts = project3to2(X,K)

pts = [K(1,1)*X(1,:)./X(3,:) + K(1,3);...
   K(2,2)*X(2,:)./X(3,:) + K(2,3)];

end