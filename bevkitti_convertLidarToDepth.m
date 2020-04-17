%% Convert Velodyne to Sparse Depth
close all
clear all


base_dir = 'H:/data_kitti_bev/2012_object/training/';
calib_dir = 'H:/data_kitti_bev/2012_object/training/calib/';

    
for ff=200:1000
    fprintf("k = %d\n", ff);
    fname = sprintf('%06d', ff);
    
    % load calibration
    [~, ~, P2, ~, R_rect, Tr_velo_to_cam, ~] = bevkitti_readCalib(strcat(calib_dir, fname, '.txt'));
    R_cam_to_rect = eye(4);
    R_cam_to_rect(1:3,1:3) = R_rect;
    P_velo_to_img = P2*R_cam_to_rect*Tr_velo_to_cam;

    % load and display image
    img = imread(strcat(base_dir, 'image_02/data/', fname, '.png'));
    % fig = figure('Position',[20 100 size(img,2) size(img,1)]); axes('Position',[0 0 1 1]);
    % imshow(img); hold on;

    % load velodyne points
    fid = fopen(strcat(base_dir, 'velodyne/', fname, '.bin'), 'rb');
    velo = fread(fid,[4 inf],'single')';
    fclose(fid);

    % remove all points behind image plane (approximation
    idx = velo(:,1) < 0.5;
    velo(idx,:) = [];

    % project to image plane (exclude luminance)
    velo_img = project(velo(:,1:3), P_velo_to_img);
    [npoints, ~] = size(velo_img);

    [height, width, ~] = size(img);
    depth = zeros(height, width);
    for k = 1:npoints
        if ((round(velo_img(k,1)) >= 1) && (round(velo_img(k,2)) >= 1) && (round(velo_img(k,1)) <= width) && (round(velo_img(k,2)) <= height))
            depth(round(velo_img(k,2)), round(velo_img(k,1))) = velo(k,1);
        end
    end
    
    imwrite(uint16(depth*256.0), strcat(base_dir, 'proj_depth/velodyne_raw/image_02/', fname, '.png'));
end
