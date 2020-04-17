%% iros2020 video

im = imread('h:/data_kitti_bev/2012_object/training/image_02/data/000107.png');
sparselabel = imread('h:/data_kitti_bev/2012_object/training/instance/sparse_colored/000107.png');
denselabel = imread('h:/data_kitti_bev/2012_object/training/instance/dense_colored/000107.png');
refinedlabel = imread('h:/data_kitti_bev/2012_object/training/instance/refined_colored/000107.png');

ims = im;
imd = im;
imr = im;

[sR, sG, sB] = imsplit(sparselabel);
[dR, dG, dB] = imsplit(denselabel);
[rR, rG, rB] = imsplit(refinedlabel);
[R, G, B] = imsplit(im);
[imsR, imsG, imsB] = imsplit(ims);
[imdR, imdG, imdB] = imsplit(imd);
[imrR, imrG, imrB] = imsplit(imr);

nosparse = (sR==255)&(sG==255)&(sB==255);
imsR(~nosparse) = sR(~nosparse);
imsG(~nosparse) = sG(~nosparse);
imsB(~nosparse) = sB(~nosparse);
imsparse = cat(3, imsR, imsG, imsB);

nodense = (dR==255)&(dG==255)&(dB==255);
imdR(~nodense) = dR(~nodense);
imdG(~nodense) = dG(~nodense);
imdB(~nodense) = dB(~nodense);
imdense = cat(3, imdR, imdG, imdB);

norefined = (rR==255)&(rG==255)&(rB==255);
imrR(~norefined) = rR(~norefined);
imrG(~norefined) = rG(~norefined);
imrB(~norefined) = rB(~norefined);
imrefined = cat(3, imrR, imrG, imrB);

% imsparse = imfuse(im, sparselabel, 'blend');
% imdense = imfuse(im, denselabel, 'blend');
% imrefined = imfuse(im, refinedlabel, 'blend');

% [height, width, ~] = size(imsparse);
% for j=1:height
%     for i=1:width
%         
%     end
% end

propgradient = imread('h:/data_kitti_bev/2012_object/training/instance/prop_gradient/000107.png');
insgradient = imread('h:/data_kitti_bev/2012_object/training/instance/instance_gradient/000107.png');

imwrite(imsparse, 'h:/data_kitti_bev/2012_object/training/imsparse.png');
imwrite(imdense, 'h:/data_kitti_bev/2012_object/training/imdense.png');
imwrite(imrefined, 'h:/data_kitti_bev/2012_object/training/imrefined.png');