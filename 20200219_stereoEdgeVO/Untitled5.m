close all;clear all;clc;


min_disp = 5;
max_disp = 64; % disparity�� �������� �ϴ°� ���ڴ�.
disp = min_disp:max_disp;
focal = 469;
baseline = 0.03;
bf = baseline*focal;
bfinv = 1/bf;
depth = bf./disp;

fprintf('min depth: %0.2f / MAX depth: %0.2f\n',  depth(end),depth(1));

figure();
plot(disp,depth,'x-');
