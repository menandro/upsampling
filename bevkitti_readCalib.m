%read calib file from kitti
function [P0, P1, P2, P3, R0_rect, Tr_velo_to_cam, Tr_imu_to_velo] = bevkitti_readCalib(path)
%     clear all
%     close all
%     path = 'H:\data_kitti_bev\2012_object\training\calib/000000.txt';
    fid = fopen(path);
    P0x = fscanf(fid, ['P0:' '%f %f %f %f %f %f %f %f %f %f %f %f' '%f']);
    P1x = fscanf(fid, ['P1:' '%f %f %f %f %f %f %f %f %f %f %f %f' '%f']);
    P2x = fscanf(fid, ['P2:' '%f %f %f %f %f %f %f %f %f %f %f %f' '%f']);
    P3x = fscanf(fid, ['P3:' '%f %f %f %f %f %f %f %f %f %f %f %f' '%f']);
    R0_rectx = fscanf(fid, ['R0_rect:' '%f %f %f %f %f %f %f %f %f' '%f']);
    Tr_velo_to_camx = fscanf(fid, ['Tr_velo_to_cam:' '%f %f %f %f %f %f %f %f %f %f %f %f' '%f']);
    Tr_imu_to_velox = fscanf(fid, ['Tr_imu_to_velo:' '%f %f %f %f %f %f %f %f %f %f %f %f' '%f']);
    
    P0 = reshape(P0x, [4,3])';%zeros(4,3);
    P1 = reshape(P1x, [4,3])';
    P2 = reshape(P2x, [4,3])';
    P3 = reshape(P3x, [4,3])';
    R0_rect = reshape(R0_rectx, [3,3])';
    Tr = reshape(Tr_velo_to_camx, [4,3])';
    Tr_velo_to_cam = [Tr; 0 0 0 1];
    Tr_imu_to_velo = reshape(Tr_imu_to_velox, [4,3])';
    
    fclose(fid);