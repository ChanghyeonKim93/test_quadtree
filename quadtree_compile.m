function quadtree_compile()
% clc
localpath = fileparts(which('quadtree_compile'));
fprintf(1,'Compiling quadtree library [%s]...\n', localpath);

err = 0;
err = err | mex('-outdir',localpath, [localpath,'/Quadtree_build_mex.cpp']);
err = err | mex('-outdir',localpath, [localpath,'/Quadtree_nearest_neighbor_mex.cpp']);
% err = err | mex('-outdir',localpath, [localpath,'/BKDTree_delete_mex.cpp']);

if err ~= 0, 
   error('compile failed!'); 
else
   fprintf(1,'\bDone!\n');
end 