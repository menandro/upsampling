close all
clear all

%%kitti
for k=5:5:426
    if k<10
        filler = '000000000';
    elseif k<100
        filler = '00000000';
    else
        filler = '0000000';
    end
    fname = strcat(filler,int2str(k));
    semanticName = fname;
    mainfolder = 'h:/data_kitti_raw/2011_09_26/2011_09_26_drive_0093_sync/';
    [resultProp, resultOpt] = upsampling_iv(fname, semanticName, mainfolder);
    
    imwrite(uint16(resultProp*256), strcat(mainfolder, 'output/upsampling/',fname, 'p.png'));
    imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling/', fname, '.png'));
end

%% faro
% for k=1:56
%     fname = strcat('im',int2str(k));
%     semanticName = strcat('sem',int2str(k));  
%     mainfolder = 'h:/data_ivdata/';
%     [resultProp, resultOpt] = upsampling_iv(fname, semanticName, mainfolder);
%     
%     imwrite(uint16(resultProp*256), strcat(mainfolder, 'output/upsampling/im', int2str(k), 'p.png'));
%     imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling/im', int2str(k), '.png'));
% end