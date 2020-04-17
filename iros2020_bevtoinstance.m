%% Create Bounding Box for sparse point cloud and dense depth
% and instance segmentation na rin
close all
clear all

%% Create Bounding Box for Sparse Point Cloud and Instance Segment
scale = 20;
fname = '0000000278';
mainfolder = 'H:/data_kitti_raw/2011_09_26/2011_09_26_drive_0093_sync/';
bevFilename = strcat(mainfolder, 'bev/image_02/', fname, '.mat');
depth = double(imread(strcat(mainfolder, 'proj_depth/velodyne_raw/image_02/', fname, '.png')))/256.0;
im = imread(strcat(mainfolder, 'image_02/data/', fname, '.png'));
[RR, GG, BB] = imsplit(im);
[height, width] = size(depth);
u = double(repmat((1:width), height,1));
v = double(repmat((1:height)', 1,width));

f = 721.5377;
cx = 609.5593;
cy = 172.8540;

x = depth.*(u - cx)/f;
y = depth.*(v - cy)/f;
z = depth;

minX = min(x(:)) - 1;
maxX = max(x(:)) + 1;
minY = min(y(:)) - 1;
maxY = max(y(:)) + 1;
minZ = min(z(:)) - 1;
maxZ = max(z(:)) + 1;
X = x(depth~=0);
Y = y(depth~=0);
Z = z(depth~=0);

R = RR(depth~=0);
G = GG(depth~=0);
B = BB(depth~=0);

XX = int32(scale*(X - minX));
YY = int32(scale*(Y - minY));
ZZ = int32(scale*(Z - minZ));

% Project points topview
topWidth = int32(scale*(maxX - minX));
topHeight = int32(scale*(maxZ - minZ));
topViewR = uint8(zeros(topHeight, topWidth));
topViewG = uint8(zeros(topHeight, topWidth));
topViewB = uint8(zeros(topHeight, topWidth));
idx = sub2ind([topHeight, topWidth], ZZ, XX);
topViewR(idx) = R;
topViewG(idx) = G;
topViewB(idx) = B;
% topView = cat(3, topViewR, topViewG, topViewB);
topView = uint8(zeros(topHeight, topWidth));
topView(idx) = 255;

%% Interactive bounding box setter
front = figure;
imshow(im);
top = figure;
imshow(topView);
% set(0, 'currentFigure', top);
topPoly = drawpolygon(top.CurrentAxes);

boxHeight = 2.0;
curPos = topPoly.Position / scale;
topBox = zeros(4, 3);
topBox(:,1) = curPos(:,1) + minX;
topBox(:,2) = boxHeight + minY;
topBox(:,3) = curPos(:,2) + minZ;

topU = f *(topBox(:,1)./topBox(:,3)) + cx;
topV = f *(topBox(:,2)./topBox(:,3)) + cy;
frontPoly = drawpolygon(front.CurrentAxes, 'Position', [topU topV], 'Color', 'r');

addlistener(topPoly, 'ROIMoved', @(src, evt)topViewMove(src, evt, frontPoly, scale, minX, minZ, f, cx, cy, topBox));
addlistener(frontPoly, 'ROIMoved', @(src, evt)frontViewMove(src, evt, frontPoly, topU, topBox, f, cy));  

% Project points frontview
% frontWidth = int32(10*(maxX - minX));
% frontHeight = int32(10*(maxY - minY));
% frontViewR = uint8(zeros(frontHeight, frontWidth));
% frontViewG = uint8(zeros(frontHeight, frontWidth));
% frontViewB = uint8(zeros(frontHeight, frontWidth));
% idx = sub2ind([frontHeight, frontWidth], YY, XX);
% frontViewR(idx) = R;
% frontViewG(idx) = G;
% frontViewB(idx) = B;
% frontView = cat(3, frontViewR, frontViewG, frontViewB);
% imshow(frontView);


pcloud = pointCloud([x(:)'; z(:)'; -y(:)']');
% pcshow(pcloud);

function frontViewMove(src, evt, varargin)
    frontPoly = varargin{1};
    topU = varargin{2};
    topBox = varargin{3};
    f = varargin{4};
    cy = varargin{5};
    evname = evt.EventName;
    switch(evname)
        case{'ROIMoved'}
%             disp(['ROI moved current position: ' mat2str(evt.CurrentPosition)]);
            curPos = evt.CurrentPosition;
            disp(['ROI moved current position: ' mat2str(topBox)]);
%             topBox(:,2) = (curPos(:,2) - cy) * topBox(:,3) / f;
            topV = curPos(:,2);
            frontPoly.Position = [topU topV];
    end
end

function topViewMove(src, evt, varargin)
    frontPoly = varargin{1};
    scale = varargin{2};
    minX = varargin{3};
    minZ = varargin{4};
    f = varargin{5};
    cx = varargin{6};
    cy = varargin{7};
    topBox = varargin{8};
    evname = evt.EventName;
    switch(evname)
        case{'ROIMoved'}
            curPos = evt.CurrentPosition / scale;
            topBox(:,1) = curPos(:,1) + minX;
            topBox(:,3) = curPos(:,2) + minZ;
            
            topU = f *(topBox(:,1)./topBox(:,3)) + cx;
            topV = f *(topBox(:,2)./topBox(:,3)) + cy;
            frontPoly.Position = [topU topV];
    end
end
