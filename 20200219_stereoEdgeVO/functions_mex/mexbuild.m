%% buill all
cd('/functions_mex')
mex -O -largeArrayDims findSalientEdges.cpp
mex -O -largeArrayDims objectpoolBuild.cpp
mex -O -largeArrayDims objectpoolDelete.cpp

mex -O -largeArrayDims quadtreeBuild.cpp
mex -O -largeArrayDims quadtreeDelete.cpp

mex -O -largeArrayDims quadtreeNN.cpp
mex -O -largeArrayDims quadtreeCachedNN.cpp

mex -O -largeArrayDims quadtreemBuild.cpp
mex -O -largeArrayDims quadtreemDelete.cpp

mex -O -largeArrayDims quadtreemNN.cpp
mex -O -largeArrayDims quadtreemCachedNN.cpp
cd ...