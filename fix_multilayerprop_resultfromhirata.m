%% Fix new propagation result
close all
clear all

k=12;
mainfolder = 'h:/data_ivdata/';
grayFilename = strcat(mainfolder, 'image_02/data/im', int2str(k), '.png');
propagationFilename = strcat(mainfolder, 'output/upsampling_iros2020/im', int2str(k), 'p.png');
propagationinsFilename = strcat(mainfolder, 'output/upsampling_iros2020/im', int2str(k), 'p_multilayer.png');

prop = double(imread(propagationFilename)/256.0);
propins = double(imread(propagationinsFilename))/256.0;

plot(propins(170,:));
hold on
plot(prop(170,:));
hold off