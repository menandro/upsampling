clear all; close all;

% options
root_dir = 'H:/data_kitti_bev/2012_object';
data_set = 'training';
img_idx = 107;

image_dir = fullfile(root_dir,[data_set '/image_02/data']);
label_dir = fullfile(root_dir,[data_set '/label_02']);
calib_dir = fullfile(root_dir,[data_set '/calib']);

im = imread(strcat(image_dir, '/', sprintf('%06d.png', img_idx)));
hh = figure;
imshow(im);

% get number of images for this dataset
nimages = length(dir(fullfile(image_dir, '*.png')));

% set up figure
% h = visualization('init', image_dir);

P = readCalibration(calib_dir, img_idx, 2);
objects = readLabels(label_dir, img_idx);
% visualization('update', image_dir, h, img_idx, nimages, data_set);

occ_col    = {'g','y','r','w'};
trun_style = {'-','--'};

% for all annotated objects do
for obj_idx=1:numel(objects)
    % plot 2D bounding box
%     drawBox2D(h, objects(obj_idx));

    % plot 3D bounding box
    [corners, face_idx] = computeBox3D(objects(obj_idx),P);
    orientation = computeOrientation3D(objects(obj_idx),P);
%     drawBox3D(h, objects(obj_idx), corners, face_idx, orientation);

    object = objects(obj_idx);
    trc = double(object.truncation>0.1)+1;
    if ~isempty(corners)
        for f=1:4
            line([corners(1, face_idx(f,:)), corners(1,face_idx(f,1))]+1,...
               [corners(2,face_idx(f,:)),corners(2,face_idx(f,1))]+1,...
               'parent', hh.CurrentAxes, 'color', occ_col{object.occlusion+1},...
               'LineWidth',3,'LineStyle',trun_style{trc});
            line([corners(1,face_idx(f,:)),corners(1,face_idx(f,1))]+1,...
               [corners(2,face_idx(f,:)),corners(2,face_idx(f,1))]+1,...
               'parent', hh.CurrentAxes, 'color','b','LineWidth',1);
        end
    end
end
