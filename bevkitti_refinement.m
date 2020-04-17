%% Segmentation Refinement KITTI
close all
clear all

img_idx = 107;
fname = sprintf('%06d', img_idx);
mainfolder = 'h:/data_kitti_bev/2012_object/training/';
label = imread(strcat(mainfolder, 'instance/dense/', fname, '.png'));
depth = double(imread(strcat(mainfolder, 'iros2020/upsampling/', fname, 'p.png')))/256.0;
boxlabel_dir = strcat(mainfolder, 'label_02');
im = imread(strcat(mainfolder, 'image_02/data/', fname, '.png'));
refinementregion = imread(strcat(mainfolder, 'instance/refinementregion/', fname, '.png'));
nolabelimg = imread(strcat(mainfolder, 'instance/sparse/', fname, 'nolabel.png'));

im = im*1.5;
gray = double(rgb2gray(im))/256.0;
gray_ = imfilter(gray, fspecial('gaussian',[3 3], 0.5));
G_x = [-1 1;
       -1 1];
G_y = G_x';
grad_x = imfilter(gray_, G_x, 'replicate');
grad_y = imfilter(gray_, G_y, 'replicate');
grad = sqrt(grad_x.^2 + grad_y.^2);

nLabels = max(label(:));
% nLabels =1;
[height, width] = size(label);

objects = readLabels(boxlabel_dir, img_idx);
% Get box center's depth
for k=1:nLabels
    boxdepth(k) = objects(k).t(3);
end

% Put everything in a list
for k=1:nLabels
    mask = false(height, width);
    mask(label==k) = k;
    labels{k} = mask;
    dsub = depth;
    dsub(labels{k}==0) = 0;
    dlabel{k} = dsub;
    meanlabel(k) = mean(dsub(labels{k}==1));
end

% Get nolabel weight
[nolabel, noindex] = bwdist(nolabelimg);
bnolabel = false(height, width);
bnolabel(nolabel > 3) = 1;
% gnolabel = 1./(1 + 0.1*nolabel);

% Label weight - calculate distance from each label
for k=1:nLabels
    [wlabel{k}, index] = bwdist(labels{k});
    idx{k} = mod(index, height);
    idy{k} = (index - idx{k})/width;
    bnolabel(labels{k}==1)= 1;
end

refreg = bnolabel;
% refreg(gnolabel > 0.7) = 1;


% Depth weight - calculate difference from mean depth of each label
for k=1:nLabels
    wdepth{k} = abs(depth - boxdepth(k));
    wdepth{k}(labels{k}==1)=1;
end

for k=1:nLabels
%     maxGrad = zeros(height, width);
%     idxx = idx{k};
%     idyy = idy{k};
% %     plot(labels{k}), pause();
%     for j=1:height
%         for i=1:width
%             %max gradient in x direction
%             maxgradx = 0.0;
%             maxgrady = 0.0;
%            
%             x = idxx(j,i);
%             y = idyy(j,i);
%             
%             if (labels{k}(j,i) == 1)
%                 maxGrad(j,i) = 0.0;
%             elseif (x==0)||(y==0)
%                 maxGrad(j,i) = 0.0;
%             else
%                 %max gradient in x direction
%                 if (i < x)
%                     for ii=i:x
%                         if (grad(j, ii) > maxgradx)
%                             maxgradx = grad(j,ii);
%                         end
%                     end
%                     %max gradient in y direction
%                     if (j < y)
%                         for jj = j:y
%                             if (grad(jj,ii) > maxgradx)
%                                 maxgradx = grad(jj,ii);
%                             end
%                         end
%                     elseif (j > y)
%                         for jj = y:j
%                             if (grad(jj,ii) > maxgradx)
%                                 maxgradx = grad(jj,ii);
%                             end
%                         end
%                     end
%                 elseif (i > x)
%                     for ii=x:i
%                         if (grad(j,ii) > maxgradx)
%                             maxgradx = grad(j,ii);
%                         end
%                     end
%                     if (j < y)
%                         for jj = j:y
%                             if (grad(jj,ii) > maxgradx)
%                                 maxgradx = grad(jj,ii);
%                             end
%                         end
%                     elseif (j > y)
%                         for jj = y:j
%                             if (grad(jj,ii) > maxgradx)
%                                 maxgradx = grad(jj,ii);
%                             end
%                         end
%                     end
%                 end
% 
%                 
% %                 maxGrad(j,i) = sqrt(maxgradx^2 + maxgrady^2);
% %                 maxGrad(j,i) = max(maxgradx, maxgrady);
%                 maxGrad(j,i) = maxgradx;
%             end
%         end
%     end
%     wimage{k} = maxGrad;
%     imwrite(uint16(maxGrad*256.0), strcat(mainfolder, 'instance/', fname, '/gradientlabel', int2str(k), '.png'));
end

for k=1:nLabels
    maxgrad = double(imread(strcat(mainfolder, 'instance/', fname, '/gradientlabel', int2str(k), '.png')))/256.0;
    maxgrad(labels{k}==1) = 0.0;
    wimage{k} = maxgrad;
end

% Color similarity instead of gradient
colorDiff = zeros(height, width);
for k=1:nLabels
    maxGrad = zeros(height, width);
    idxx = idx{k};
    idyy = idy{k};
    for j=1:height
        for i=1:width
            x = idxx(j,i);
            y = idyy(j,i);
            if (labels{k}(j,i) == 1)
                colorDiff(j,i) = 0.0;
            elseif (x==0)||(y==0)
                colorDiff(j,i) = 0.0;
            else
                colorDiffR = abs(double(im(j,i,1)) - double(im(y,x,1)));
                colorDiffG = abs(double(im(j,i,2)) - double(im(y,x,2)));
                colorDiffB = abs(double(im(j,i,3)) - double(im(y,x,3)));
                colorDiff(j,i) = sqrt(colorDiffR^2 + colorDiffG^2 + colorDiffB^2);
            end
        end
    end
    wcolor{k} = colorDiff;
end

for k=1:nLabels
    wtotal{k} = false(height, width);
end

for k=1:nLabels
    glabel{k} = 1./(1 + 0.001*wlabel{k});
    gdepth{k} = 1./(1 + 0.1*wdepth{k});
    gimage{k} = 1./(1 + 1*wimage{k});
    gcolor{k} = 1./(1 + 1*wcolor{k});
%     wtotal{k} = glabel.*gimage;
    wtotal{k} = glabel{k}.*gdepth{k}.*gimage{k};
%     wtotal{k} = glabel{k}.*gimage{k};
%     wtotal{k}(wtotal{k}==1) = 0;
end

refmanual = false(height, width);
refmanual(refinementregion(:,:,1)==0 & refinementregion(:,:,2)==255 & refinementregion(:,:,3)==0) = true;
for k=1:nLabels
    refval = refreg | refmanual;
%     refval = true(height, width);
    for comp=1:nLabels
        if (k~=comp)
            refval = refval & (wtotal{k} > wtotal{comp});
        end
    end
    refval(wlabel{k} > 10) = false;
    refval(wtotal{k} < 0.3) = false;
%     refval(wtotal{k} < 0.018) = false;
    ref{k} = refval;
end


% Relabel
labelimg = zeros(height, width);
for k=1:nLabels
    labelimg(ref{k}) = k;
end

imwrite(labelimg, strcat(mainfolder, '/instance/refined/', sprintf('%06d.png', img_idx)));

% labelimgdil = imdilate(labelimg, strel('diamond', 2));
labelimgvis = ind2rgb(uint8(256*normalization(labelimg, 'default' , double(numel(objects)), 0.0)), prism(256));
[labelimgvisR, labelimgvisG, labelimgvisB] = imsplit(labelimgvis);
labelimgvisR(labelimg==0) = 255;
labelimgvisG(labelimg==0) = 255;
labelimgvisB(labelimg==0) = 255;
labelimgvis = cat(3, labelimgvisR, labelimgvisG, labelimgvisB);
imwrite(labelimgvis, strcat(mainfolder, '/instance/refined_colored/', sprintf('%06d.png', img_idx)));

imshow(imfuse(im, labelimgvis, 'blend'));
% figure, imshow(imfuse(im, label, 'blend'));
figure
% plot(labels{1}(332,:))
% hold on
yline = 266;
labelline = 2;
plot(wtotal{labelline}(yline,:), 'r')
hold on
plot(gdepth{labelline}(yline,:), 'g')
plot(glabel{labelline}(yline,:), 'b')
% plot(gcolor{1}(332,:), 'm')
plot(gimage{labelline}(yline,:), 'm')
% plot(nolabel(yline,:), 'c');
hold off
figure, imshow(ref{labelline});
% figure, imshow(gimage{labelline})
% figure, imshow(glabel{labelline})
figure, imshow(labelimgvis);
figure, imshow(refreg);














