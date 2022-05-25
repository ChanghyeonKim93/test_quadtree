function data_info = loadStereoData(dataset_name,subset_name,root_dir, flag_illumination)
tic;
data_info = struct('dataset_name','-',...
   'subset_name','-',...
   'leftnames','-',...
   'rightnames','-',...
   'time','-',...
   'intrin',[],...
   'extrin',[],...
   'rectify',[],...
   'imu',[],...
   'gt',[]);

if(strcmp(dataset_name , 'euroc'))
   %% 
   addpath('euroc_tools');
   addpath('euroc_tools/quaternion');
   
   % set dataset folder
   datasetPath = [root_dir,'\euroc\',subset_name];
   
   disp(' ');
   disp([' > dataset_load_test [', datasetPath, ']']);
   
   assert(exist(datasetPath, 'dir') > 0, ...
      ' > Dataset folder does not exist, Please set datasetPath.');
   
   % load dataset
   data_euroc = dataset_load(datasetPath);
   
   % extrinsics
   % imu 를 body frame으로 표현되어있다.
   extrin.g_il = data_euroc.body{1}.sensor{1}.T_BS;
   extrin.g_ir = data_euroc.body{1}.sensor{2}.T_BS;
   extrin.g_iv = data_euroc.body{1}.sensor{6}.T_BS; % imu to vicon.
   %    extrin.g_clcr = extrin.g_ic^-1*(extrin.g_il^-1*extrin.g_ir)*extrin.g_ic;
%    extrin.g_clcr = (extrin.g_ic^-1*extrin.g_il)^-1*extrin.g_ic^-1*extrin.g_ir;
   extrin.g_clcr = extrin.g_il^-1*extrin.g_ir;
   
   data_info.extrin = extrin;
   
   % Ground truth
   if(length(data_euroc.body{1}.sensor) > 5)
      g_iv0 = data_euroc.body{1}.sensor{6}.T_BS; % IMU to Vicon reference frame. maybe initial vicon pose.
      g_cv0 = extrin.g_il^-1*g_iv0;
      T_v0v = data_euroc.body{1}.sensor{6}.data.p_RS_R;
      q_v0v = data_euroc.body{1}.sensor{6}.data.q_RS; % order: qw qx qy qz
      g_cv  = zeros(4,4,length(q_v0v));
      
      for i=1:length(q_v0v)
         g_cv(:,:,i) = g_cv0*[quat_to_dcm_kch(q_v0v(:,i)),T_v0v(:,i);0,0,0,1];
      end
   end
   
   
   % dataset names and etc.
   data_info.dataset_name = dataset_name;
   data_info.subset_name  = subset_name;
   data_info.leftnames    = data_euroc.body{1}.sensor{1}.data.filenames;
   data_info.rightnames   = data_euroc.body{1}.sensor{2}.data.filenames;
   data_info.time         = double(data_euroc.body{1}.sensor{1}.data.t)*1e-9; % seconds.
   fprintf('Total length of dataset: %d\n',length(data_info.leftnames));
   if(length(data_euroc.body{1}.sensor) > 5)
      
      time_v = double(data_euroc.body{1}.sensor{6}.data.t)*1e-9; % seconds.
      % GT interpolation to make GT fit into camera timestamps.
      g_cv_cam = zeros(4,4,length(data_info.time));
      time_c = data_info.time;
      for i = 1:length(time_c)
         q_cur = [interp1(time_v,q_v0v(1,:),time_c(i));...
            interp1(time_v,q_v0v(2,:),time_c(i));...
            interp1(time_v,q_v0v(3,:),time_c(i));...
            interp1(time_v,q_v0v(4,:),time_c(i))];
         T_cur = [interp1(time_v,T_v0v(1,:),time_c(i));...
            interp1(time_v,T_v0v(2,:),time_c(i));...
            interp1(time_v,T_v0v(3,:),time_c(i))];
         g_cv_cam(:,:,i) = [quat_to_dcm_kch(q_cur),T_cur;0,0,0,1]*g_cv0^-1;
      end
      data_info.gt.g = g_cv_cam;
      data_info.gt.t = time_v;
      invalid_idx = find( or(data_info.time > time_v(end), data_info.time < time_v(1)));
   else
      invalid_idx = [];
   end
   data_info.leftnames(invalid_idx)  = [];
   data_info.rightnames(invalid_idx) = [];
   data_info.time(invalid_idx)       =[];
   
   
   if(flag_illumination == false)
      for i = 1:length(data_info.leftnames)
         data_info.leftnames{i}  = [root_dir,'\euroc\',subset_name,'/mav0/cam0/data/',data_info.leftnames{i}];
         data_info.rightnames{i} = [root_dir,'\euroc\',subset_name,'/mav0/cam1/data/',data_info.rightnames{i}];
      end
   else
      for i = 1:length(data_info.leftnames)
         data_info.leftnames{i}  = [root_dir,'\euroc\',subset_name,'/mav0/cam0/data_illumination/',data_info.leftnames{i}];
         data_info.rightnames{i} = [root_dir,'\euroc\',subset_name,'/mav0/cam1/data_illumination/',data_info.rightnames{i}];
      end
   end
   
   % left caemra
   fu = data_euroc.body{1}.sensor{1}.intrinsics{1};
   fv = data_euroc.body{1}.sensor{1}.intrinsics{2};
   centu = data_euroc.body{1}.sensor{1}.intrinsics{3};
   centv = data_euroc.body{1}.sensor{1}.intrinsics{4};
   intrin.K.left = [fu,0,centu;0,fv,centv;0,0,1];
   d1 = data_euroc.body{1}.sensor{1}.distortion_coefficients{1};
   d2 = data_euroc.body{1}.sensor{1}.distortion_coefficients{2};
   d3 = data_euroc.body{1}.sensor{1}.distortion_coefficients{3};
   d4 = data_euroc.body{1}.sensor{1}.distortion_coefficients{4};
   intrin.distortion.left = [d1,d2,d3,d4,0];
   intrin.n_cols.left = data_euroc.body{1}.sensor{1}.resolution{1};
   intrin.n_rows.left = data_euroc.body{1}.sensor{1}.resolution{2};
   
   % right camera
   fu = data_euroc.body{1}.sensor{2}.intrinsics{1};
   fv = data_euroc.body{1}.sensor{2}.intrinsics{2};
   centu = data_euroc.body{1}.sensor{2}.intrinsics{3};
   centv = data_euroc.body{1}.sensor{2}.intrinsics{4};
   intrin.K.right = [fu,0,centu;0,fv,centv;0,0,1];
   d1 = data_euroc.body{1}.sensor{2}.distortion_coefficients{1};
   d2 = data_euroc.body{1}.sensor{2}.distortion_coefficients{2};
   d3 = data_euroc.body{1}.sensor{2}.distortion_coefficients{3};
   d4 = data_euroc.body{1}.sensor{2}.distortion_coefficients{4};
   intrin.distortion.right = [d1,d2,d3,d4,0];
   intrin.n_cols.right = data_euroc.body{1}.sensor{2}.resolution{1};
   intrin.n_rows.right = data_euroc.body{1}.sensor{2}.resolution{2};
   data_info.intrin = intrin;
   
   % plot dataset
   %    dataset_plot(dataset);
   

elseif(strcmp(dataset_name , 'kitti'))
   %% 
   datasetPath = [root_dir,'\kitti\',subset_name];
   
   disp(' ');
   disp([' > dataset_load_test [', datasetPath, ']']);
   
   for i = 1:n_total
      rgb_name{k} = [dir,'/image_00/data/',sprintf('%010d',k),'.png'];

   end
   
   
  
elseif(strcmp(dataset_name , 'custom'))
   %% 
   % set dataset folder
   %  L stereo -- association_stereo.txt
   %            L left  - images.
   %            L right - images.
   addpath('euroc_tools');
   addpath('euroc_tools/quaternion');
   datasetPath = [root_dir,'\custom\',subset_name];
   
   disp(' ');
   disp([' > dataset_load_test [', datasetPath, ']']);
   
   assert(exist(datasetPath, 'dir') > 0, ...
      ' > Dataset folder does not exist, Please set datasetPath.');
   
   assoc_stereo = importdata([datasetPath,'\stereo\association_stereo.txt']);
   n_img = length(assoc_stereo)-1; % # of images.
   
   
   % dataset ground truth
%    truth_data = importdata([datasetPath,'\groundtruth.txt']);
%    if(~isempty(truth_data.data))
%       t_vicon = truth_data.data(:,1);
%       t_vicon = t_vicon - t_vicon(1);
%       trans_vicon = truth_data.data(:,2:4)';
%       trans_vicon = trans_vicon - trans_vicon(:,1);
%       q_vicon = truth_data.data(:,5:8)';
%       
%       figure('position',[40,0,600,400]);
%       subplot(3,1,1); plot(t_vicon, trans_vicon(1,:),'k'); grid minor; ylabel('x [m]'); ylim([-1,1]*5);title('Position');
%       subplot(3,1,2); plot(t_vicon, trans_vicon(2,:),'k'); grid minor; ylabel('y [m]'); ylim([-1,1]*5);
%       subplot(3,1,3); plot(t_vicon, trans_vicon(3,:),'k'); grid minor; ylabel('z [m]'); ylim([-1,1]*5);
%       
%       figure();
%       plot3(trans_vicon(1,:), trans_vicon(2,:), trans_vicon(3,:),'k','linewidth',2);grid minor; hold on;
%       plot3(trans_vicon(1,1),trans_vicon(2,1),trans_vicon(3,1),'m+','linewidth',2);
%       plot3(trans_vicon(1,end),trans_vicon(2,end),trans_vicon(3,end),'g*');
%       
%       A = [1,0,0,1;0,0,0,1;0,1,0,1;0,0,0,1;0,0,1,1;0,0,0,1]';
%       L = 0.06; % [m];
%       A = [A(1:3,:)*L;A(4,:)];
%       R_vc = [0,0,1;-1,0,0;0,-1,0];
%       for i = 1:20:size(trans_vicon,2)
%          g_temp = [q2r(q_vicon(:,i))*R_vc,trans_vicon(:,i);0,0,0,1];
%          B = g_temp*A;
%          plot3(B(1,1:2),B(2,1:2),B(3,1:2),'r','linewidth',1.5);
%          plot3(B(1,3:4),B(2,3:4),B(3,3:4),'g','linewidth',1.5);
%          plot3(B(1,5:6),B(2,5:6),B(3,5:6),'b','linewidth',1.5);
%       end
%       xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');title('trajectory');axis equal;
% 
%       
%       
%       data_info.gt.trans = trans_vicon;
%       data_info.gt.q = q_vicon;
%       data_info.gt.t = t_vicon;
%    end

   % dataset names and etc.
   data_info.dataset_name = dataset_name;
   data_info.subset_name  = subset_name;
   data_info.leftnames    = cell(n_img,1);
   data_info.rightnames   = cell(n_img,1);
   data_info.time         = zeros(n_img,1); % seconds.
   
   %    invalid_idx = find( or(data_info.time > time_v(end), data_info.time < time_v(1)));
   %    data_info.leftnames(invalid_idx)  = [];
   %    data_info.rightnames(invalid_idx) = [];
   %    data_info.time(invalid_idx)       = [];
   for i = 1:1:n_img
      data_info.leftnames{i}  = [datasetPath,'\stereo\left\',assoc_stereo{i+1}(1:17),'.png'];
      data_info.rightnames{i} = [datasetPath,'\stereo\right\',assoc_stereo{i+1}(1:17),'.png'];
      data_info.time(i)       = str2num(assoc_stereo{i+1}(1:17));
   end

   fprintf('Total length of dataset: %d\n',n_img);
   
   assoc_imu    = importdata([root_dir,'\custom\',subset_name,'\imu.txt']);
   if(~isempty(assoc_imu) && length(assoc_imu) > 1)
      fprintf('IMU data is detected.\n');
      
      time_imu = assoc_imu.data(:,1);
      acc_imu  = assoc_imu.data(:,2:4);
      gyro_imu = assoc_imu.data(:,5:7);
      q_imu    = assoc_imu.data(:,11:14);
      
      data_info.imu.t          = time_imu;
      data_info.imu.acc        = acc_imu';
      data_info.imu.gyro       = gyro_imu';
      data_info.imu.quaternion = q_imu';
   else
      
   end
   
   % intrinsic parameters
   left_yaml    = [datasetPath,'\..\left_ocams_kalibr.yaml'];
   left_params  = ReadYaml(left_yaml);  % a function from euroc_tools
   if(isempty(left_params) == true)
      fprintf(' there is no left yaml file.\n');
   end
   data_info.intrin.K.left = ...
      [left_params.camera_matrix.data{1},left_params.camera_matrix.data{2},left_params.camera_matrix.data{3};...
      left_params.camera_matrix.data{4},left_params.camera_matrix.data{5},left_params.camera_matrix.data{6};...
      left_params.camera_matrix.data{7},left_params.camera_matrix.data{8},left_params.camera_matrix.data{9}];
      
   data_info.intrin.distortion.left = ...
      [left_params.distortion_coefficients.data{1},...
      left_params.distortion_coefficients.data{2},...
      left_params.distortion_coefficients.data{3},...
      left_params.distortion_coefficients.data{4},...
      left_params.distortion_coefficients.data{5}];
   data_info.intrin.n_cols.left = left_params.image_width;
   data_info.intrin.n_rows.left = left_params.image_height;
   right_yaml   = [datasetPath,'\..\right_ocams_kalibr.yaml'];
   right_params = ReadYaml(right_yaml); % a function  from euroc_tools
   if(isempty(right_params) == true)
      fprintf(' there is no right yaml file.\n');
   end
   data_info.intrin.K.right = ...
      [right_params.camera_matrix.data{1},right_params.camera_matrix.data{2},right_params.camera_matrix.data{3};...
      right_params.camera_matrix.data{4},right_params.camera_matrix.data{5},right_params.camera_matrix.data{6};...
      right_params.camera_matrix.data{7},right_params.camera_matrix.data{8},right_params.camera_matrix.data{9}];
   data_info.intrin.distortion.right = ...
      [right_params.distortion_coefficients.data{1},...
      right_params.distortion_coefficients.data{2},...
      right_params.distortion_coefficients.data{3},...
      right_params.distortion_coefficients.data{4},...
      right_params.distortion_coefficients.data{5}];
   data_info.intrin.n_cols.right = right_params.image_width;
   data_info.intrin.n_rows.right = right_params.image_height;
   1;
   
   % Extrinsic parameters(oCamS)
   extrinsic_yaml = [datasetPath,'\..\extrinsic_ocams_kalibr.yaml'];
   ext_param = ReadYaml(extrinsic_yaml);  % a function from euroc_tools
   if(isempty(ext_param) == true)
      fprintf(' there is no yaml file for extrinsic parameter.\n');
   end
   
   % Matlab notation : X_c^T = X_w^T*R + t (row vector)   
   % My notation : X_c = R^-1*X_w + t^T
   %               R*X_c - R*t^T = X_w 
   % Therefore, R = R_12, -R*t^T = T_12.
   % where R = rotation matrix from world to camera w.r.t.
   % world.(row-major)
   % t = translation row vector from world to camera w.r.t world
   % 
   % See https://kr.mathworks.com/help/vision/ref/extrinsics.html
   R_clcr = ...
      [ext_param.extrinsic_parameters.R_lr{1},ext_param.extrinsic_parameters.R_lr{2},ext_param.extrinsic_parameters.R_lr{3};...
       ext_param.extrinsic_parameters.R_lr{4},ext_param.extrinsic_parameters.R_lr{5},ext_param.extrinsic_parameters.R_lr{6};...
       ext_param.extrinsic_parameters.R_lr{7},ext_param.extrinsic_parameters.R_lr{8},ext_param.extrinsic_parameters.R_lr{9}];
    %    t_matlab = [ext_param.extrinsic_parameters.T_lr{1},ext_param.extrinsic_parameters.T_lr{2},ext_param.extrinsic_parameters.T_lr{3}]';
    %    T_clcr = -R_clcr*t_matlab;
    
    T_clcr = [ext_param.extrinsic_parameters.T_lr{1},ext_param.extrinsic_parameters.T_lr{2},ext_param.extrinsic_parameters.T_lr{3}]';

   data_info.extrin.g_il =...
      [1,0,0,0.0024;...
      0,1,0,0.0289;...
      0,0,1,0.0006;...
      0,0,0,1];
   data_info.extrin.g_ir =...
      [1,0,0,0.0024;...
      0,1,0,-0.0911;...
      0,0,1,0.0006;...
      0,0,0,1];
   data_info.extrin.g_ic = [[0,0,1;-1,0,0;0,-1,0],[0,0,0]';0,0,0,1];
   data_info.extrin.g_clcr = [R_clcr,T_clcr;0,0,0,1]; % 이렇게 변환하는거였네 ;;;

   1;
else
   assert(false,'There is no such dataset name!\n');
end

dt = toc;
fprintf('Elapsed time for data loading: %0.3f [s]\n',dt); 
end