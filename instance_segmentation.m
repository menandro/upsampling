% Instance segmentation from BEV
close all
clear all

% im = imread('h:/data_ivdata/proj_depth/velodyne_raw/image_02/im12.png');
% im = double(im)/256.0;
% imshow(im, [0 40], 'ColorMap', jet);
% imcolor = ind2rgb(uint8(255.0*normalization(im, 'default', 40.0, 0.0)), jet(256));
% imwrite(imcolor, 'h:/data_ivdata/instance/mask12.png');

% Convert mask to instance labels
mask = imread('h:/data_ivdata/instance/mask12.png');
[height, width, ch] = size(mask);
rmask = mask(:,:,1);
gmask = mask(:,:,2);
bmask = mask(:,:,3);
labels = zeros(height, width);
labels(rmask==255 & gmask==0 & bmask==0) = 1;
labels(rmask==128 & gmask==0 & bmask==0) = 5;
labels(rmask==128 & gmask==128 & bmask==0) = 8;
labels(rmask==255 & gmask==128 & bmask==0) = 4;
labels(rmask==255 & gmask==255 & bmask==0) = 3;
labels(rmask==255 & gmask==0 & bmask==255) = 6;
labels(rmask==0 & gmask==0 & bmask==255) = 2;
labels(rmask==0 & gmask==255 & bmask==255) = 9;
labels(rmask==0 & gmask==128 & bmask==255) = 7;
labels(rmask==128 & gmask==128 & bmask==255) = 10;
labels(rmask==128 & gmask==0 & bmask==128) = 0;
labels(rmask==255 & gmask==255 & bmask==255) = 0;
% labels(labels==0) = 12;
% imshow(labels, [0 11], 'ColorMap', jet);

% Create instance point cloud
depth = imread('h:/data_ivdata/proj_depth/velodyne_raw/image_02/im12.png');
depth = double(depth)/256.0;
se = strel('diamond', 1);
depth = imdilate(depth, se);
instance = zeros(height, width);
instance(depth ~= 0) = labels(depth~=0);
instance(labels == 0) = 0;
imwrite(instance, 'h:/data_ivdata/instance/instance12.png');


% Instance Segmentation Gradient v2
% R = [255 128 128 255 255 255 0 0 0 128];
% G = [0 0 128 128 255 0 0 255 128 128];
% B = [0 0 0 0 0 255 255 255 255 255];
% 
% for k=1:10
%     
% end

% Instance Segmentation Gradient
insgt = imread('h:/data_ivdata/instance/instance_full.png');
insgtR = insgt(:,:,1);
insgtG = insgt(:,:,2);
insgtB = insgt(:,:,3);
insR = uint8(zeros(height, width));
insG = uint8(zeros(height, width));
insB = uint8(zeros(height, width));
insR(insgtR==255 & insgtG == 0 & insgtB==0) = 255;
insR(insgtR==128 & insgtG == 0 & insgtB==0) = 128;
insR(insgtR==128 & insgtG == 128 & insgtB==0) = 128; insG(insgtR==128 & insgtG == 128 & insgtB==0) = 128;
insR(insgtR==255 & insgtG == 128 & insgtB==0) = 255; insG(insgtR==255 & insgtG == 128 & insgtB==0) = 128;
insR(insgtR==255 & insgtG == 255 & insgtB==0) = 255; insG(insgtR==255 & insgtG == 255 & insgtB==0) = 255;
insR(insgtR==255 & insgtG == 0 & insgtB==255) = 255; insB(insgtR==255 & insgtG == 0 & insgtB==255) = 255;
insB(insgtR==0 & insgtG == 0 & insgtB==255) = 255;
insG(insgtR==0 & insgtG == 255 & insgtB==255) = 255; insB(insgtR==0 & insgtG == 255 & insgtB==255) = 255;
insG(insgtR==0 & insgtG == 128 & insgtB==255) = 128; insB(insgtR==0 & insgtG == 128 & insgtB==255) = 255;
insR(insgtR==128 & insgtG == 128 & insgtB==255) = 128; insG(insgtR==128 & insgtG == 128 & insgtB==255) = 128; insB(insgtR==128 & insgtG == 128 & insgtB==255) = 255;
insImage = cat(3, insR, insG, insB);
figure, imshow(insImage);
filtSize = 3;
G_x = [-1 0 1];
G_y = G_x';
insR = double(insR)/256.0;
insG = double(insG)/256.0;
insB = double(insB)/256.0;
% insR = imgaussfilt(double(insR)/256.0, filtSize);
% insG = imgaussfilt(double(insG)/256.0, filtSize);
% insB = imgaussfilt(double(insB)/256.0, filtSize);
insGradR = 4*sqrt(imfilter(insR, G_x, 'replicate').^2 + imfilter(insR, G_y, 'replicate').^2);
insGradG = 4*sqrt(imfilter(insG, G_x, 'replicate').^2 + imfilter(insG, G_y, 'replicate').^2);
insGradB = 4*sqrt(imfilter(insB, G_x, 'replicate').^2 + imfilter(insB, G_y, 'replicate').^2);
% insGrad = normalization(insGradR) + normalization(insGradG) + normalization(insGradB);
insGrad = (1 - normalization(insGradR)) .* (1 - normalization(insGradG)) .* (1 - normalization(insGradB));
% insGrad = (1 - insGradR) .* (1 - insGradG) .* (1 - insGradB);
insGrad = 1-insGrad;

% insImageGray = double(rgb2gray(insImage))/256.0;
% insImageGray = normalization(insImageGray);
% insImageGray = imgaussfilt(insImageGray, 3);
% G_x = [-1 0 1];
% G_y = G_x';
% insGradX = imfilter(insImageGray, G_x, 'replicate');
% insGradY = imfilter(insImageGray, G_y, 'replicate');
% insGrad = 4*sqrt(insGradX.^2 + insGradY.^2);
insGrad(insGrad > 0.3) = 1.0; 
insGrad = imgaussfilt(insGrad, 3, 'FilterSize', 41);
figure, plot(insGrad(:,200));
insGrad = normalization(insGrad, 'default', 0.2, 0);
insGrad(insGrad > 1.0) = 1.0;
insGrad = 1 - insGrad;
% figure, imshow(insGrad, [0 0.3], 'ColorMap', jet);
figure, imshow(insGrad);
figure, plot(insGrad(:,200));
imwrite(uint16(insGrad*256.0), 'h:/data_ivdata/instance/insGrad.png');

% % Nearest neighborhood
% windowSize = 20;
% instancefilt = ordfilt2(instance, windowSize^2, true(windowSize));
% % instancefilt = imgaussfilt(instancefilt, 1);
% figure, imshow(instancefilt);
% % instancefilt = imfilter(instance, ones(windowSize,windowSize)/(windowSize*windowSize));
% figure, imshow(instancefilt, [0, 12], 'ColorMap', jet);
% instancefilt = imgaussfilt(25*instancefilt, 1);
% % figure, imshow(instancefilt);
% G_x = [-1 0 1];
% G_y = G_x';
% insGradX = imfilter(instancefilt, G_x, 'replicate');
% insGradY = imfilter(instancefilt, G_y, 'replicate');
% insGrad = sqrt(insGradX.^2 + insGradY.^2);
% % insGrad = normalization(insGrad, 'default', 1, 0);
% % insGrad(insGrad > 1.0) = 1.0; 
% % insGrad = bwmorph(insGrad, 'shrink', 20);
% figure, imshow(insGrad);

% For displaying
% instancecolor = 255.0*uint8(ind2rgb(uint8(255.0*normalization(instance, 'default', 11.0, 0.0)), jet(256)));
instancecolor = uint8(zeros(size(mask)));
instancecolorR = instancecolor(:,:,1);
instancecolorG = instancecolor(:,:,2);
instancecolorB = instancecolor(:,:,3);
instancecolorR(:) = 0;
instancecolorG(:) = 0;
instancecolorB(:) = 0;
instancecolorR(depth~=0 & labels~=0) = rmask(depth~=0 & labels~=0);
instancecolorG(depth~=0 & labels~=0) = gmask(depth~=0 & labels~=0);
instancecolorB(depth~=0 & labels~=0) = bmask(depth~=0 & labels~=0);
% instancecolorR(depth~=0 ) = rmask(depth~=0 );
% instancecolorG(depth~=0 ) = gmask(depth~=0 );
% instancecolorB(depth~=0 ) = bmask(depth~=0 );
instancecolor = cat(3, instancecolorR, instancecolorG, instancecolorB);
imwrite(instancecolor, 'h:/data_ivdata/instance/instancecolor.png');
% figure, imshow(instancecolor);
% figure, imshow(cat(3, rmask, gmask, bmask));