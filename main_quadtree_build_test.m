close all;clear all;clc;
%% 20190310_quadtree test
% (1) build 

% points
npoints = 1000;
pts = randn(npoints,2)*20+200;
queries = pts+1;
cached_nodes = ones(npoints,1);

% image size
n_rows = 480;
n_cols = 640;

% quadtree build
max_depth = 4;
eps = 0.1;
tic;
quadtree_ptr = Quadtree_build_mex(pts, n_rows, n_cols, max_depth, eps);
toc
cached_nodes = quadtree_ptr*cached_nodes;

figure();
plot(pts(:,1),pts(:,2),'r.');hold on;
plot(queries(:,1),queries(:,2),'b.');
%%
[idxes, cached_nodes] = Quadtree_nearest_neighbor_mex(quadtree_ptr, queries, cached_nodes);