function q= r2q(R)
% Project:   Patch-based illumination-variant DVO
% Function: r2q
%
% Description:
%   This function convert rotation matrix to unit orientation quaternion
%
% Example:
%   OUTPUT:
%   quatVector: quaternion vector composed of [qw qx qy qz]
%
%   INPUT:
%   rotMtx = Rotation Matrix [3x3]
%               defined as [Inertial frame] = rotMtxBody * [Body frame]
%
% NOTE:
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2015-02-06: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

qw = sqrt( (1+R(1,1)^2+R(2,2)^2 +R(3,3)^2)/4);
qx = (R(3,2)-R(2,3))/4/qw;
qy = (R(1,3)-R(3,1))/4/qw;
qz = (R(2,1) - R(1,2))/4/qw;

q=[qw,qx,qy,qz].';
q=q/norm(q);
end

