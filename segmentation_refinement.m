%% Segmentation Refinement
close all
clear all

im = imread('h:/data_ivdata/image_02/data/im12.png');
prop = double(imread('h:/data_ivdata/output/upsampling_iros2020/im12p_multilayer.png'))/256.0;
sem = imread('h:/data_ivdata/semantic/sem12.png');

% depth gradient
G_x = [-1 1; -1 1];
G_y = G_x';
propGradX = imfilter(prop, G_x, 'replicate');
propGradY = imfilter(prop, G_y, 'replicate');
propGrad = sqrt(propGradX.^2 + propGradY.^2);
propGrad = normalization(propGrad, 'default', 5, 0);
propGrad(propGrad >= 0.98) = 0.98;
alpha1 = 1.2 * (1 - propGrad);

% prop = normalization(prop);
% propGrad = (1 - get_edge_sed(cat(3, prop, prop, prop)));
% propGrad = 2*(1 - get_edge_sed(cat(3, prop, double(im(:,:,1))/256.0, double(im(:,:,2))/256.0)));

imGrad = (1-get_edge_sed(im));
figure, imshow(2*propGrad, [0 1], 'ColorMap', jet);
figure, imshow(2*imGrad, [0 1], 'ColorMap', jet);
figure, imshow(2*propGrad.*imGrad, [0 1], 'ColorMap', jet);

% semantic gradient
semgray = double(rgb2gray(sem))/256.0;
G_x = [-1 1; -1 1];
G_y = G_x';
semGradX = imfilter(semgray, G_x, 'replicate');
semGradY = imfilter(semgray, G_y, 'replicate');
semGrad = sqrt(semGradX.^2 + semGradY.^2);
semGrad = normalization(semGrad, 'default', 0.1, 0);
semGrad(semGrad >= 0.98) = 0.98;
semGrad = imdilate(semGrad, strel('diamond', 2));
alpha1 = 1.2 * (1 - semGrad);
% figure, imshow(semGrad);

semR = double(sem(:,:,1))/256.0;
semG = double(sem(:,:,2))/256.0;
semB = double(sem(:,:,3))/256.0;
resR = (1-propGrad) .* double(semR);
resG = (1-propGrad) .* double(semG);
resB = (1-propGrad) .* double(semB);
semRef = cat(3, resR, resG, resB);

c1 = imfuse(im, semRef, 'blend');
figure, imshow(c1);
c2 = imfuse(im, sem, 'blend');
figure, imshow(c2);
