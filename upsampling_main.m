close all
clear all

%% namita
% for k=0:0
%     if k<10
%         filler = '000000000';
%     elseif k<100
%         filler = '00000000';
%     else
%         filler = '0000000';
%     end
%     fname = strcat(filler,int2str(k));
%     semanticName = fname;
%     mainfolder = 'h:/data_namita_rover/';
%     [resultProp, resultOpt] = upsampling_ppdeigsb(fname, semanticName, mainfolder);
%     
%     imwrite(uint16(resultProp*256), strcat(mainfolder, 'output/upsampling/',fname, 'p.png'));
%     imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling/', fname, '.png'));
% end

%% kitti
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
%kk = [12 8 18 29 35 47];
% kk = 12;
% for j=1:1
%     k = kk(j);
%     disp(k);
%     fname = strcat('im',int2str(k));
%     semanticName = strcat('sem',int2str(k));  
%     mainfolder = 'h:/data_ivdata/';
%     %[resultProp, resultOpt] = upsampling_iv(fname, semanticName, mainfolder);
%     [resultProp, resultOpt] = upsampling_pp(fname, semanticName, mainfolder);
%     
%     imwrite(uint16(resultProp*256), strcat(mainfolder, 'output/upsampling_eachterm/pp', int2str(k), 'p.png'));
%     imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling_eachterm/pp', int2str(k), '.png'));
%     
%     %vis
%     %resultPropJet = ind2rgb(normalization(resultProp,'default', 40.0, 0.0), jet(256));
%     
% end

%% SAVING PER FRAME PROPAGATION
% kk = [12 8 18 29 35 47];
% for j=1:1
%     k = kk(j);
%     disp(k);
%     fname = strcat('im',int2str(k));
%     semanticName = strcat('sem',int2str(k));  
%     mainfolder = 'h:/data_ivdata/';
%     [resultProp, resultOpt] = upsampling_iv_save(fname, semanticName, mainfolder);
%     
%     imwrite(uint16(resultProp*256), strcat(mainfolder, 'output/upsampling_video/im', int2str(k), 'p.png'));
%     imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling_video/im', int2str(k), '.png'));
%     
%     %vis
%     %resultPropJet = ind2rgb(normalization(resultProp,'default', 40.0, 0.0), jet(256));
%     
% end