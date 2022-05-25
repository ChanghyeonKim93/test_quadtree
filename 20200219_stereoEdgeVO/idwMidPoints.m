function X = idwMidPoints(pt1, pt2, g_21)
global data_info 
Kinv = data_info.rectify.Kinv;

% Brief  : Inverse depth weighted midpoint triangulation w.r.t. {pts2} frame
R_21 = g_21(1:3,1:3);
t_21 = g_21(1:3,4);
R21Kinv = R_21*Kinv;

Rf1 = R21Kinv*[pt1;1];
Rf1 = Rf1/norm(Rf1);
f2  = Kinv*[pt2;1];
f2  = f2/norm(f2);

% lambdas
invnormcross_Rf1f2 = 1/norm(cross(Rf1,f2));
lam1 = norm(cross(f2,t_21))  *invnormcross_Rf1f2;
lam2 = norm(cross(Rf1,t_21)) *invnormcross_Rf1f2;

invlam1 = 1/lam1;
invlam2 = 1/lam2;
X = (invlam1*t_21 + Rf1 + f2)/(invlam1 + invlam2);

end