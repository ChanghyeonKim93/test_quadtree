function pts_patch = calcEpiPatch(pt, line, line_normal, win_sz, fat_sz)
win_len = win_sz*2+1;
fat_len = fat_sz*2+1;
pts_patch = zeros(2,fat_len*win_len);

%           + l_normal 방향
%             ^
%  28 29 30 31| 33 34 35 36
%  19 20 21 22| 24 25 26 27
% ---------------------------> + l 방향.
%  10 11 12 13  15 16 17 18
%   1  2  3  4   6  7  8  9
pts_patchstrip = pt + line.*(-win_sz:win_sz);
for f = 1:fat_len
   pts_patch(:,(1+(f-1)*win_len):(f*win_len)) = pts_patchstrip + line_normal*(f-1-fat_sz);
end
end