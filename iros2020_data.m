close all
clear all

%% Save Gradients
k=12;
fname = strcat('im',int2str(k));
mainfolder = 'h:/data_ivdata/';
propagationFilename = strcat(mainfolder, 'output/upsampling_iros2020/im', int2str(k), 'p_multilayer.png');
instanceGradFilename = strcat(mainfolder, 'instance/insGrad.png');
prop = imread(propagationFilename);
prop = double(prop)/256.0;
insgrad = imread(instanceGradFilename);
insgrad = double(insgrad)/256.0;
G_x = [-1 1; -1 1];
G_y = G_x';
propGradX = imfilter(prop, G_x, 'replicate');
propGradY = imfilter(prop, G_y, 'replicate');
propGrad = sqrt(propGradX.^2 + propGradY.^2);
propGrad = normalization(propGrad, 'default', 1, 0);
propGrad(propGrad >= 0.98) = 0.98;
% Inverse Instance and Gradient
%     alpha1 = 1.2 * (1 - (propGrad.*(1 - insgrad)));
%     alpha0 = 17*(1 - (propGrad.*(1 - insgrad)));
% Instance and Gradient
alpha1 = 1.2 * (1 - propGrad).*(insgrad);
imwrite(propGrad, 'h:/data_ivdata/output/upsampling_iros2020/refinement/propgradient.png');
imwrite(1-insgrad, 'h:/data_ivdata/output/upsampling_iros2020/refinement/instancegradient.png');

%% Convert depth to color
% filename = 'new_opt_instanceonly_multilayer';
% depth = double(imread(strcat('h:/data_ivdata/output/upsampling_iros2020/im12/', filename, '.png')))/256.0;
% depthvis = ind2rgb(uint8(255.0*normalization(depth, 'default', 40.0, 0.0)), jet(256));
% depthvisfilename = strcat('h:/data_ivdata/output/upsampling_iros2020/im12/vis/', filename, '.png');
% imshow(depthvis);
% imwrite(depthvis, depthvisfilename);

%% RMSE colored error
% filename = 'new_opt_multilayer';
% errorFilename = strcat('h:/data_ivdata/output/upsampling_iros2020/error_jet/', filename, '.png');
% depth = double(imread(strcat('h:/data_ivdata/output/upsampling_iros2020/im12/', filename, '.png')))/256.0;
% gt = double(imread('h:/data_ivdata/proj_depth/groundtruth/image_02/im12.png'))/256.0;
% % imshow(gt, [0 40], 'ColorMap', jet);
% rmse = abs((depth-gt)./depth);
% rmse(gt==0)=0;
% % imshow(rmse, [0 3], 'ColorMap', jet);
% rmseColor = ind2rgb(uint8(255.0*normalization(rmse, 'default', 0.05, 0.0)), jet(256));
% imwrite(rmseColor, errorFilename);
% % imshow(rmseColor);

%% Resample pointcloud for showing in the paper
% gt = imread('h:/data_ivdata/proj_depth/groundtruth/image_02/im12.png');
% gt = double(gt)/256.0;
% [height, width] = size(gt);
% dout = zeros(height, width);
% sparsity = 8;
% for j=1:height
%     for i=1:width
%         if (mod(j, sparsity)==0) && (mod(i, sparsity)==0)
%             if (gt(j,i) > 0)
%                 dout(j,i) = gt(j,i);
%             end
%         end
%     end
% end
% dout = imdilate(dout, strel('diamond', 2));
% dcolor = ind2rgb(uint8(255.0*normalization(dout, 'default', 40.0, 0)), jet(256));
% imwrite(dcolor, 'h:/data_ivdata/proj_depth/velodyne_raw/image_02/vis/im12.png');