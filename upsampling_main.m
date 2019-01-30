close all
clear all

for k=1:56
    fname = strcat('im',int2str(k));
    semanticName = strcat('sem',int2str(k));  
    mainfolder = 'h:/data_ivdata/';
    [resultProp, resultOpt] = upsampling_iv(fname, semanticName, mainfolder);
    
    imwrite(uint16(resultProp*256), strcat(mainfolder, 'output/upsampling/im', int2str(k), 'p.png'));
    imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling/im', int2str(k), '.png'));
end