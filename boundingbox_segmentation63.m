%% Bounding Box Segmentation for Segmentation Refinement
% And Segmentation Refinement na rin
close all
clear all

preply = 'h:/data_ivdata/output/upsampling_iros2020/refinement/instance_prerefinement63.ply';
preinstance = 'h:/data_ivdata/output/upsampling_iros2020/refinement/instance_prerefinement63.png';

postply = 'h:/data_ivdata/output/upsampling_iros2020/refinement/instance_postrefinement63.ply';
postinstance = 'h:/data_ivdata/output/upsampling_iros2020/refinement/instance_postrefinement63.png';

refinementregion = imread('h:/data_ivdata/output/upsampling_iros2020/refinement/refinementregion263.png');

preinstancefuse = 'h:/data_ivdata/output/upsampling_iros2020/refinement/instance_prerefinementfuse63.png';
postinstancefuse = 'h:/data_ivdata/output/upsampling_iros2020/refinement/instance_postrefinementfuse63.png';

depth = double(imread('h:/data_ivdata/output/upsampling_iros2020/im12p_multilayer.png'))/256.0;
[height, width] = size(depth);
im = imread('h:/data_ivdata/image_02/data/im12.png');
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

R = im(:,:,1);
G = im(:,:,2);
B = im(:,:,3);


% Detect First Car
% imshow(z);
car1 = true(height, width);
% car1 = car1 & (y < y(lowY, lowX) & y > y(highY, highX));
car1 = car1 & (x < -3.1818 & x > -7);
car1 = car1 & (y < 1.3652 & y > -0.0523);
car1 = car1 & (z > 8.2508 & z < 10.0);
boxdepth(1) = (8.2508 + 10.0)/2;
R(car1) = 255;
G(car1) = 0;
B(car1) = 0;

% Detect Second Car
% imshow(z);
car2 = true(height, width);
% car1 = car1 & (y < y(lowY, lowX) & y > y(highY, highX));
car2 = car2 & (x > -8 & x < -2.55);
car2 = car2 & (y > -0.2 & y < 1.17);
car2 = car2 & (z > 13.4 & z < 15.3);
boxdepth(2) = (13.4 + 15.3)/2;
R(car2) = 0;
G(car2) = 255;
B(car2) = 0;

% Detect Car 3
% imshow(z);
car3 = true(height, width);
car3 = car3 & (x > -7.695 & x < -3.5036);
car3 = car3 & (y > -1.53 & y < 1.00);
car3 = car3 & (z > 24.4 & z < 26.2);
boxdepth(3) = (24.4 + 26.2)/2;
R(car3) = 255;
G(car3) = 255;
B(car3) = 0;

% Detect Car 4
% imshow(z);
car4 = true(height, width);
car4 = car4 & (x > 4& x < 7);
car4 = car4 & (y > 0.1 & y < 1);
car4 = car4 & (z > 10 & z < 12);
boxdepth(4) = (10 + 12)/2;
R(car4) = 0;
G(car4) = 128;
B(car4) = 128;

% % Detect Car 5
% % imshow(z);
car5 = true(height, width);
car5 = car5 & (x > 3.8 & x < 7);
car5 = car5 & (y > -0.2 & y < 1.00);
car5 = car5 & (z > 13.6 & z < 15.5);
boxdepth(5) = (13.6 + 15.5)/2;
R(car5) = 255;
G(car5) = 128;
B(car5) = 0;
% 
% % Detect Car 6
% % imshow(z);
car6 = true(height, width);
car6 = car6 & (x > 3.6 & x < 7);
car6 = car6 & (y > -0.2 & y < 1.00);
car6 = car6 & (z > 16.4 & z < 18);
boxdepth(6) = (16.4 + 18)/2;
R(car6) = 255;
G(car6) = 0;
B(car6) = 255;

% Detect Wall
% imshow(z);
wall = true(height, width);
wall = wall & (x > -10.695 & x < -1);
wall = wall & (y > -2.1 & y < 1.00);
wall = wall & (z > 30.6 & z < 30.8);
boxdepth(7) = (30.6 + 30.8)/2;
R(wall) = 0;
G(wall) = 128;
B(wall) = 255;

imout = cat(3, R, G, B);
% figure, imshow(imout);
imwrite(imout, preinstance);
imwrite(imfuse(im, cat(3, R, G, B), 'blend'), preinstancefuse);

color = [R(:)'; G(:)'; B(:)']';
pcloud = pointCloud([x(:)'; z(:)'; -y(:)']', 'Color', color);
% figure, pcshow(pcloud);
% pcwrite(pcloud, preply);


%% Segmentation Refinement
% Put everything in a list
labels = {car1, car2, car3, car4, car5, car6, wall};
for k=1:7
    dsub = depth;
    dsub(labels{k}==0) = 0;
    dlabel{k} = dsub;
    meanlabel(k) = mean(dsub(labels{k}==1));
end

% Label weight - calculate distance from each label
for k=1:7
    [wlabel{k}, index] = bwdist(labels{k});
    idx{k} = mod(index, height);
    idy{k} = (index - idx{k})/width;
end

% Depth weight - calculate difference from mean depth of each label
for k=1:7
    wdepth{k} = abs(depth - boxdepth(k));
    wdepth{k}(labels{k}==1)=0;
end

% Maximum gradient between the pixel and the label
% gray_ = imfilter(gray, fspecial('gaussian',[3 3], 0.5));
% G_x = [-1 1;
%        -1 1];
% G_y = G_x';
% grad_x = imfilter(gray_, G_x, 'replicate');
% grad_y = imfilter(gray_, G_y, 'replicate');
% grad = sqrt(grad_x.^2 + grad_y.^2);
% 
% for k=1:7
%     fprintf("%d\n", k);
%     maxGrad = zeros(height, width);
%     parfor j=1:height
%         fprintf("%d\n", j);
%         for i=1:width
%             c = improfile(grad, [i double(idx{k}(j,i))], [j double(idy{k}(j,i))]);
%             maxGrad(j,i) = max(c(:));
%         end
%     end
%     wimage{k} = maxGrad;
% end
for k=1:7
    wimage{k} = double(imread(strcat('h:/data_ivdata/instance/gradientlabel', int2str(k), '.png')))/256.0;
%     wimage{k}(labels{k}==1)=0;
end

% plot(wlabel{1}(:,175));
% hold on
% plot(2*wdepth{1}(:,175));
% plot(10*wimage{1}(:,175));

% Calculate weights per label
for k=1:7
    wtotal{k} = false(height, width);
end

for k=1:7
    glabel = 1./(1 + wlabel{k});
    gdepth = 1./(1 + 2*wdepth{k});
    gimage = wimage{k};%1./(1 + 1*wimage{k});
%     wtotal{k} = glabel.*gimage;
    wtotal{k} = glabel.*gdepth.*gimage;
%     wtotal{k}(wtotal{k}==1) = 0;
end
% plot(glabel(:,241))
% hold on
% plot(gdepth(:,241))
% plot(gimage(:,241))
% plot(wtotal{k}(:,241));

% Get maximum weight per label
refreg = false(height, width);
refreg(refinementregion(:,:,1)==0 & refinementregion(:,:,2)==255 & refinementregion(:,:,3)==0) = true;
ref1 = (refreg) & (wtotal{1} > wtotal{2}) & (wtotal{1} > wtotal{3}) & (wtotal{1} > wtotal{4}) & (wtotal{1} > wtotal{5}) & (wtotal{1} > wtotal{6}) & (wtotal{1} > wtotal{7});
ref2 = (refreg) & (wtotal{2} > wtotal{1}) & (wtotal{2} > wtotal{3}) & (wtotal{2} > wtotal{4}) & (wtotal{2} > wtotal{5}) & (wtotal{2} > wtotal{6}) & (wtotal{2} > wtotal{7});
ref3 = (refreg) & (wtotal{3} > wtotal{1}) & (wtotal{3} > wtotal{2}) & (wtotal{3} > wtotal{4}) & (wtotal{3} > wtotal{5}) & (wtotal{3} > wtotal{6}) & (wtotal{3} > wtotal{7});
ref4 = (refreg) & (wtotal{4} > wtotal{1}) & (wtotal{4} > wtotal{2}) & (wtotal{4} > wtotal{3}) & (wtotal{4} > wtotal{5}) & (wtotal{4} > wtotal{6}) & (wtotal{4} > wtotal{7});
ref5 = (refreg) & (wtotal{5} > wtotal{1}) & (wtotal{5} > wtotal{2}) & (wtotal{5} > wtotal{3}) & (wtotal{5} > wtotal{4}) & (wtotal{5} > wtotal{6}) & (wtotal{5} > wtotal{7});
ref6 = (refreg) & (wtotal{6} > wtotal{1}) & (wtotal{6} > wtotal{2}) & (wtotal{6} > wtotal{3}) & (wtotal{6} > wtotal{4}) & (wtotal{6} > wtotal{5}) & (wtotal{6} > wtotal{7});
ref7 = (refreg) & (wtotal{7} > wtotal{1}) & (wtotal{7} > wtotal{2}) & (wtotal{7} > wtotal{3}) & (wtotal{7} > wtotal{4}) & (wtotal{7} > wtotal{5}) & (wtotal{7} > wtotal{6});
% c = imfuse(ref4, im, 'blend');
% imshow(c)

% Combine then threshold
% ref = zeros(height, width);
% total = zeros(height, width);
% for k=1:7
%     ref = ref + wtotal{k}.*k*10;
%     total = total + wtotal{k};
% end
% outlabel = ref./total;
% outlabel(~refreg) = 0;
% % outlabel = round(outlabel);
% ref7 = refreg & (outlabel < 75);
% ref6 = refreg & (outlabel < 65);
% ref5 = refreg & (outlabel < 55);
% ref4 = refreg & (outlabel < 45);
% ref3 = refreg & (outlabel < 35);
% ref2 = refreg & (outlabel < 25);
% ref1 = refreg & (outlabel < 15);

% Write new label
%car1
R(ref1) = 255;
G(ref1) = 0;
B(ref1) = 0;

%car2
R(ref2) = 0;
G(ref2) = 255;
B(ref2) = 0;

%car3
R(ref3) = 255;
G(ref3) = 255;
B(ref3) = 0;

%car4
R(ref4) = 0;
G(ref4) = 128;
B(ref4) = 128;

%car5
R(ref5) = 255;
G(ref5) = 128;
B(ref5) = 0;

%car6
R(ref6) = 255;
G(ref6) = 0;
B(ref6) = 255;

%wall
R(ref7) = 0;
G(ref7) = 128;
B(ref7) = 255;

refinedLabel = cat(3,R,G,B);
imwrite(refinedLabel, postinstance);
fused = imfuse(im, refinedLabel, 'blend');
imshow(fused);
imwrite(imfuse(im, cat(3, R, G, B), 'blend'), postinstancefuse);




















% combine = car1 | car2 | car3 | car4 | car5 | car6 | wall;
% nolabel = ~combine;
% c = imfuse(im, combine, 'blend');
% % imshow(c);
% 
% % Threshold the depth wrt to the mask
% dcar1 = depth;
% dcar1(car1==0) = 0;
% meancar1 = mean(dcar1(car1==1));
% 
% dnolabel = depth;
% dnolabel(nolabel==0) = 0;
% imshow(dnolabel, [0 40], 'ColorMap', jet);

% Distance from each label image


% Nearest neighbor filling with geodesic distance
% radius = 5;
% for j=(1+radius):(height-radius)
%     for i=(1+radius):(width-radius)
%         if dnolabel(j,i)
%             for y=-radius:radius
%                 for x=-radius:radius
%                     % Calculate geodesic distance
%                     for k=1:7
%                         
%                     end
%                 end
%             end
%         end
%     end
% end

% Thresholding



























