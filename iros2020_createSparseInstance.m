%% Convert manual label to sparse depth label
clear all
close all

fname = '0000000278';
nLabels = 8;
mainfolder = 'H:/data_kitti_raw/2011_09_26/2011_09_26_drive_0093_sync/';%'h:/data_ivdata/';
sparseLabelOut = strcat(mainfolder, 'instance/sparse/', fname, '.png');
sparseLabelColoredOut = strcat(mainfolder, 'instance/sparse_colored/', fname, '.png');
sparseLabelColoredVisOut = strcat(mainfolder, 'instance/sparse_colored/vis/', fname, '.png');

labelColor = [
    255 0 0;
    128 0 0;
    128 128 0;
    255 128 0;
    255 255 0;
    255 0 255;
    0 128 255;
    0 0 255;
    0 255 255;
    128 128 255;
    128 0 128;
    ];

% Convert manual label to mask
manual = imread(strcat(mainfolder, 'instance/manual_labeled/', fname, '.png'));
[manR, manG, manB] = imsplit(manual);
mask = uint8(zeros(size(manual)));
[maskR, maskG, maskB] = imsplit(mask);
for k=1:nLabels
    isThisLabel = manR==labelColor(k,1) & manG==labelColor(k,2) & manB==labelColor(k,3);
    maskR(isThisLabel) = manR(isThisLabel);
    maskG(isThisLabel) = manG(isThisLabel);
    maskB(isThisLabel) = manB(isThisLabel);
end
mask = cat(3, maskR, maskG, maskB);

% Create sparse instance label
sparse = double(imread(strcat(mainfolder, 'proj_depth/velodyne_raw/image_02/', fname, '.png')))/256.0;
sparseLabel = uint8(zeros(size(sparse)));
sparseColor = uint8(zeros(size(mask)));
[spcR, spcG, spcB] = imsplit(sparseColor);
sparseColorVis = uint8(zeros(size(mask))) + 255;
[spvR, spvG, spvB] = imsplit(sparseColorVis);

hasValue = sparse~=0;
hasValueBig = hasValue;
hasValueBig = imdilate(hasValueBig, strel('diamond', 1));
% imshow(sparse, [0 10]);
for k=1:nLabels
    isThisLabel = maskR==labelColor(k,1) & maskG==labelColor(k,2) & maskB==labelColor(k,3);
    
    sparseLabel(isThisLabel & hasValue) = k;
    spcR(isThisLabel & hasValue) = maskR(isThisLabel & hasValue);
    spcG(isThisLabel & hasValue) = maskG(isThisLabel & hasValue);
    spcB(isThisLabel & hasValue) = maskB(isThisLabel & hasValue);
    
    spvR(isThisLabel & hasValueBig) = maskR(isThisLabel & hasValueBig);
    spvG(isThisLabel & hasValueBig) = maskG(isThisLabel & hasValueBig);
    spvB(isThisLabel & hasValueBig) = maskB(isThisLabel & hasValueBig);
end
sparseColor = cat(3, spcR, spcG, spcB);
sparseColorVis = cat(3, spvR, spvG, spvB);
imwrite(sparseLabel, sparseLabelOut);
imwrite(sparseColor, sparseLabelColoredOut);
imwrite(sparseColorVis, sparseLabelColoredVisOut);


































