close all
clear all

%% iros2020 bev-kitti dataset
kk = [107];
for j=1:1
    k = kk(j);
    disp(k);
    img_idx = k;
    fname = sprintf('%06d', img_idx);
    mainfolder = 'h:/data_kitti_bev/2012_object/training/';
    grayFilename = strcat(mainfolder, 'image_02/data/', fname, '.png');
    propagationFilename = strcat(mainfolder, 'iros2020/upsampling/', fname, 'p.png');
    insgradFilename = strcat(mainfolder, 'instance/instance_gradient/', fname, '.png');
%     resultOpt = upsampling_optimization_old(grayFilename, propagationFilename);
    resultOpt = upsampling_optimization(grayFilename, propagationFilename, insgradFilename, 200);
    imwrite(uint16(resultOpt*256), strcat(mainfolder, 'iros2020/upsampling/', fname, '_new_opt.png'));
    
    resultPropJet = ind2rgb(uint8(255*normalization(resultOpt,'default', 40.0, 0.0)), jet(256));
    imwrite(resultPropJet, strcat(mainfolder, 'iros2020/upsampling_vis/', fname, '_new_opt.png'));
    %vis
    %resultPropJet = ind2rgb(normalization(resultProp,'default', 40.0, 0.0), jet(256));
end

%% iros2020 optimization only kitti
% kk = [12 8 18 29 35 47];
% for j=1:1
%     k = kk(j);
%     disp(k);
%     fname = '0000000278';
%     mainfolder = 'H:\data_kitti_raw\2011_09_26\2011_09_26_drive_0093_sync/';
%     grayFilename = strcat(mainfolder, 'image_02/data/', fname, '.png');
%     propagationFilename = strcat(mainfolder, 'iros2020/upsampling/', fname, 'p.png');
%     insgradFilename = strcat(mainfolder, 'instance/insGrad.png');
%     resultOpt = upsampling_optimization_old(grayFilename, propagationFilename);
% %     resultOpt = upsampling_optimization(grayFilename, propagationFilename, insgradFilename, 100);
%     imwrite(uint16(resultOpt*256), strcat(mainfolder, 'iros2020/upsampling/', fname, '_old_opt.png'));
%     
%     %vis
%     %resultPropJet = ind2rgb(normalization(resultProp,'default', 40.0, 0.0), jet(256));
% end

%% iros2020 instance segmentation/propagation
% kk = [12 8 18 29 35 47];
% for j=1:1
%     k = kk(j);
%     disp(k);
%     fname = strcat('im',int2str(k));
%     semanticName = strcat('sem',int2str(k));  
%     mainfolder = 'h:/data_ivdata/';
%     [resultProp, ~] = upsampling_instance(fname, semanticName, mainfolder);
%     
% %     imwrite(uint16(resultProp*256), strcat(mainfolder, 'output/upsampling_instance/im', int2str(k), 'p.png'));
% %     imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling_instance/im', int2str(k), '.png'));
%     
%     %vis
%     %resultPropJet = ind2rgb(normalization(resultProp,'default', 40.0, 0.0), jet(256));
% end

%% iros2020 new optimization method / optimization only
% kk = [12 8 18 29 35 47];
% for j=1:1
%     k = kk(j);
%     disp(k);
%     fname = strcat('im',int2str(k));
%     mainfolder = 'h:/data_ivdata/';
%     grayFilename = strcat(mainfolder, 'image_02/data/im', int2str(k), '.png');
%     propagationFilename = strcat(mainfolder, 'output/upsampling_iros2020/im', int2str(k), 'p_multilayer.png');
%     insgradFilename = strcat(mainfolder, 'instance/insGrad.png');
%     resultOpt = upsampling_optimization(grayFilename, propagationFilename, insgradFilename, 100);
%     imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling_iros2020/im', int2str(k), 'new_opt100_multilayer.png'));
%     
%     %vis
%     %resultPropJet = ind2rgb(normalization(resultProp,'default', 40.0, 0.0), jet(256));
% end

%% iros2020 comparison
% kk = [12 8 18 29 35 47];
% for j=1:6
%     k = kk(j);
%     disp(k);
%     fname = strcat('im',int2str(k));
%     semanticName = strcat('sem',int2str(k));  
%     mainfolder = 'h:/data_ivdata/';
%     [resultProp, resultOpt] = upsampling_ivnoms(fname, semanticName, mainfolder);
% %     [resultProp, resultOpt] = upsampling_iv(fname, semanticName, mainfolder);
%     %[resultProp, resultOpt] = upsampling_pp(fname, semanticName, mainfolder);
%     
%     imwrite(uint16(resultProp*256), strcat(mainfolder, 'output/upsampling_iros2020/im', int2str(k), 'p.png'));
%     imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling_iros2020/im', int2str(k), '.png'));
%     
%     %vis
%     %resultPropJet = ind2rgb(normalization(resultProp,'default', 40.0, 0.0), jet(256));
% end

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
% for k=5:5:426
%     if k<10
%         filler = '000000000';
%     elseif k<100
%         filler = '00000000';
%     else
%         filler = '0000000';
%     end
%     fname = strcat(filler,int2str(k));
%     semanticName = fname;
%     mainfolder = 'h:/data_kitti_raw/2011_09_26/2011_09_26_drive_0093_sync/';
%     [resultProp, resultOpt] = upsampling_iv(fname, semanticName, mainfolder);
%     
%     imwrite(uint16(resultProp*256), strcat(mainfolder, 'output/upsampling/',fname, 'p.png'));
%     imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling/', fname, '.png'));
% end

%% faro
% kk = [12 8 18 29 35 47];
% for j=1:6
%     k = kk(j);
%     disp(k);
%     fname = strcat('im',int2str(k));
%     semanticName = strcat('sem',int2str(k));  
%     mainfolder = 'h:/data_ivdata/';
%     [resultProp, resultOpt] = upsampling_iv(fname, semanticName, mainfolder);
%     %[resultProp, resultOpt] = upsampling_pp(fname, semanticName, mainfolder);
%     
%     imwrite(uint16(resultProp*256), strcat(mainfolder, 'output/upsampling_newlidar/im', int2str(k), 'p.png'));
%     imwrite(uint16(resultOpt*256), strcat(mainfolder, 'output/upsampling_newlidar/im', int2str(k), '.png'));
%     
%     %vis
%     %resultPropJet = ind2rgb(normalization(resultProp,'default', 40.0, 0.0), jet(256));
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