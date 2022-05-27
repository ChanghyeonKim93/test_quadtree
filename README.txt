valgrind --tool=cachegrind --branch-sim=yes --cachegrind-out-file=/home/larrkchdesk/Documents/quadtree/cachegrind.txt ~/Documents/quadtree/build/test_moving_matching 

kcachegrind 
cg_annotate --auto=yes Documents/quadtree/cachegrind.txt Documents/quadtree/build/test_moving_matching 

