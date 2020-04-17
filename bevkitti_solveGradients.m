%% Solve Gradients for optimization
close all
clear all

img_idx = 107;
fname = sprintf('%06d', img_idx);
mainfolder = 'h:/data_kitti_bev/2012_object/training/';
prop = double(imread(strcat(mainfolder, 'iros2020/upsampling/', fname, 'p.png')))/256.0;

% Solve instance gradient
insImage = imread(strcat(mainfolder, '/instance/refined_colored/', fname, '.png'));
figure, imshow(insImage);
filtSize = 3;
G_x = [-1 0 1];
G_y = G_x';
[insR, insG, insB] = imsplit(insImage);
insR = double(insR)/256.0;
insG = double(insG)/256.0;
insB = double(insB)/256.0;
insGradR = 4*sqrt(imfilter(insR, G_x, 'replicate').^2 + imfilter(insR, G_y, 'replicate').^2);
insGradG = 4*sqrt(imfilter(insG, G_x, 'replicate').^2 + imfilter(insG, G_y, 'replicate').^2);
insGradB = 4*sqrt(imfilter(insB, G_x, 'replicate').^2 + imfilter(insB, G_y, 'replicate').^2);
insGrad = (1 - normalization(insGradR)) .* (1 - normalization(insGradG)) .* (1 - normalization(insGradB));
insGrad = 1-insGrad;

insGrad(insGrad > 0.3) = 1.0; 
insGrad = imgaussfilt(insGrad, 3, 'FilterSize', 41);
% figure, plot(insGrad(:,200));
insGrad = normalization(insGrad, 'default', 0.2, 0);
insGrad(insGrad > 1.0) = 1.0;
insGrad = 1 - insGrad;

% Solve propagation gradient
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
alpha1 = 1.2 * (1 - propGrad).*(insGrad);
imwrite(1-propGrad, strcat(mainfolder, 'instance/prop_gradient/', fname, '.png'));
imwrite(insGrad, strcat(mainfolder, 'instance/instance_gradient/', fname, '.png'));












