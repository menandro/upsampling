% Convert depth to ply

outply = 'h:/data_ivdata/proj_depth/velodyne_raw_ply/image_02/im63.ply';
depth = double(imread('h:/data_ivdata/proj_depth/velodyne_raw/image_02/im63.png'))/256.0;

[height, width] = size(depth);
im = imread('h:/data_ivdata/image_02/data/im63.png');
im = im*1.5;
gray = double(rgb2gray(im))/256.0;

u = double(repmat((1:width), height,1));
v = double(repmat((1:height)', 1,width));

f = 942.6262;
cx = 621;
cy = 187.5;

x = depth.*(u - cx)/f;
y = depth.*(v - cy)/f;
z = depth;

color = [R(:)'; G(:)'; B(:)']';
pcloud = pointCloud([x(:)'; z(:)'; -y(:)']');
pcwrite(pcloud, outply);