function pts_patch = calcEpiPatchTemplate(pt, line, pts_patch_template)
%           + l_normal 방향
%             ^
%  28 29 30 31| 33 34 35 36
%  19 20 21 22| 24 25 26 27
% ---------------------------> + l 방향.
%  10 11 12 13  15 16 17 18
%   1  2  3  4   6  7  8  9
R = [line(1),-line(2);line(2),line(1)];
pts_patch = pt + R*pts_patch_template;
end