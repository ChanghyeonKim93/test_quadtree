function weight_bin = initializeWeightBin(weight_bin,n_bins_u,n_bins_v)
global data_info
n_cols = data_info.intrin.n_cols.left;
n_rows = data_info.intrin.n_rows.left;

if(isempty(weight_bin))
   % Make container.
   weight_bin = struct('weight',true(n_bins_v,n_bins_u),...
      'h_bound',0,'w_bound',0,...
      'n_bins_v',n_bins_v,'n_bins_u',n_bins_u);
   weight_bin.weight = true(n_bins_v,n_bins_u);
   
   % Save ALL boundary coordinates crossing across each bin.
   w_bound = zeros(1,n_bins_u+1);
   h_bound = zeros(1,n_bins_v+1);
   
   w_bound(1) = 1;
   h_bound(1) = 1;
   for i = 1:n_bins_u
      w_bound(i+1) = floor(n_cols/n_bins_u*i);
   end
   for i = 1:n_bins_v
      h_bound(i+1) = floor(n_rows/n_bins_v*i);
   end
   weight_bin.h_bound = h_bound;
   weight_bin.w_bound = w_bound;
   
   fprintf(' Weight is initialized.\n');
end
end