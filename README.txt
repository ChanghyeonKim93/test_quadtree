valgrind --tool=cachegrind --branch-sim=yes --cachegrind-out-file=/home/larr/Documents/quadtree/cachegrind.txt ~/Documents/quadtree/build/test_max_elem 

kcachegrind 
cg_annotate --auto=yes Documents/quadtree/cachegrind.txt Documents/quadtree/build/test_max_elem 


perf stat ./{EXECUTABLE_FILE}