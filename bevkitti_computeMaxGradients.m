close all
clear all

img_idx = 107;
fname = sprintf('%06d', img_idx);
mainfolder = 'h:/data_kitti_bev/2012_object/training/';
label = imread(strcat(mainfolder, 'instance/dense/', fname, '.png'));
depth = double(imread(strcat(mainfolder, 'iros2020/upsampling/', fname, 'p.png')))/256.0;
boxlabel_dir = strcat(mainfolder, 'label_02');
im = imread(strcat(mainfolder, 'image_02/data/', fname, '.png'));
gray = double(rgb2gray(im))/256.0;

nLabels = max(label(:));
[height, width] = size(label);

gray_ = imfilter(gray, fspecial('gaussian',[3 3], 0.5));
G_x = [-1 1;
       -1 1];
G_y = G_x';
grad_x = imfilter(gray_, G_x, 'replicate');
grad_y = imfilter(gray_, G_y, 'replicate');
grad = sqrt(grad_x.^2 + grad_y.^2);

for k=1:nLabels
    mask = false(height, width);
    mask(label==k) = k;
    labels{k} = mask;
    dsub = depth;
    dsub(labels{k}==0) = 0;
    dlabel{k} = dsub;
    meanlabel(k) = mean(dsub(labels{k}==1));
end

for k=1:nLabels
    [wlabel{k}, index] = bwdist(labels{k});
    idx{k} = mod(index, height);
    idy{k} = (index - idx{k})/width;
end

% for k=1:nLabels
%     fprintf("k=%d\n", k);
%     idxx = idx{k};
%     idyy = idy{k};
%     maxGrad = zeros(height, width);
%     parfor j=1:height
%         fprintf("%d\n", j);
%         for i=1:width
%             c = improfile(grad, [i double(idxx(j,i))], [j double(idyy(j,i))]);
%             maxGrad(j,i) = max(c(:));
%         end
%     end
%     wimage{k} = maxGrad;
%     imwrite(uint16(maxGrad*256.0), strcat(mainfolder, 'instance/', fname, '/gradientlabel', int2str(k), '.png'));
% end

% maxGrad = zeros(height, width);
% for k=1:nLabels
%     idxx = idx{k};
%     idyy = idy{k};
%     for j=1:height
%         for i=1:width
%             %max gradient in x direction
%             maxgradx = 0.0;
%             maxgrady = 0.0;
%            
%             x = idxx(j,i);
%             y = idyy(j,i);
%             
%             if (x==0)||(y==0)
%                 maxGrad(j,i) = 0.0;
%             else
%                 %max gradient in x direction
%                 if (i < x)
%                     for ii=i:x
%                         if (grad(j, ii) > maxgradx)
%                             maxgradx = grad(j,ii);
%                         end
%                     end
%                 elseif (i > x)
%                     for ii=x:i
%                         if (grad(j,ii) > maxgradx)
%                             maxgradx = grad(j,ii);
%                         end
%                     end
%                 end
% 
%                 %max gradient in y direction
%                 if (j < y)
%                     for jj = j:y
%                         if (grad(jj,i) > maxgrady)
%                             maxgrady = grad(jj,i);
%                         end
%                     end
%                 elseif (j > y)
%                     for jj = y:j
%                         if (grad(jj,i) > maxgrady)
%                             maxgrady = grad(jj,i);
%                         end
%                     end
%                 end
%                 maxGrad(j,i) = max(maxgradx, maxgrady);
%             end
%         end
%     end
%     imwrite(uint16(maxGrad*256.0), strcat(mainfolder, 'instance/', fname, '/gradientlabel', int2str(k), '.png'));
% end