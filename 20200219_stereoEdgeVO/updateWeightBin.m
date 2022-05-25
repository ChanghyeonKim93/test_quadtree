function weight_bin = updateWeightBin(weight_bin,pts)
% weight_bin
%  L weight_bin.weight  = matrix
%  L weight_bin.h_bound = boundary values along the v axis.
%  L weight_bin.w_bound = boundary values along the u axis.
global data_info
n_cols = data_info.intrin.n_cols.left;
n_rows = data_info.intrin.n_rows.left;

n_pts = length(pts);
% columnize.
if(size(pts,1) > size(pts,2))
   pts = pts';
end
h_bound = weight_bin.h_bound;
w_bound = weight_bin.w_bound;

% initialize bin & weight
bin_pts = zeros(weight_bin.n_bins_v,weight_bin.n_bins_u);
weight_bin.weight = false(weight_bin.n_bins_v,weight_bin.n_bins_u);

% Count how many fts in each bin.
for i = 1:n_pts
   h_idx = binarySearch(pts(2,i),h_bound);
   w_idx = binarySearch(pts(1,i),w_bound);
   
   % binary search for find included bin.
   bin_pts(h_idx,w_idx) = bin_pts(h_idx,w_idx) + 1;
end

for w = 1:weight_bin.n_bins_u
   for h = 1:weight_bin.n_bins_v
      if(bin_pts(h,w) < 1)
         weight_bin.weight(h,w) = true;
      else
         weight_bin.weight(h,w) = false;
      end
   end
end

end


function idx_cur = binarySearch(value,bounds)
idx_low  = 1;
idx_high = length(bounds);

while(idx_low <= idx_high)
   if(abs(idx_low - idx_high) == 1)
      break;
   end
   
   idx_mid = floor((idx_low + idx_high)/2);
   
   if(value >= bounds(idx_mid))
      idx_low = idx_mid;
   elseif(value < bounds(idx_mid))
      idx_high = idx_mid;
   end
   %    fprintf('low mid high : %d %0.0f %d\n',bounds(idx_low),(value),bounds(idx_high));
end
idx_cur = idx_low;
end