%input image
function resultOpt = upsampling_optimization(grayFilename, propagationFilename, instanceGradFilename, maxIteration)
%     close all
%     clear all
%     k=12;
%     fname = strcat('im',int2str(k));
%     mainfolder = 'h:/data_ivdata/';
%     grayFilename = strcat(mainfolder, 'image_02/data/im', int2str(k), '.png');
%     propagationFilename = strcat(mainfolder, 'output/upsampling_iros2020/im', int2str(k), 'p_multilayer.png');
%     instanceGradFilename = strcat(mainfolder, 'instance/insGrad.png');
    
    gray = im2double(rgb2gray(imread(grayFilename)));
    prop = imread(propagationFilename);
    prop = double(prop)/256.0;
    insgrad = imread(instanceGradFilename);
    insgrad = double(insgrad)/256.0;
%     figure, imshow(insgrad);
    [M,N] = size(prop);
    
    % Find alpha1
    G_x = [-1 1; -1 1];
    G_y = G_x';
    propGradX = imfilter(prop, G_x, 'replicate');
    propGradY = imfilter(prop, G_y, 'replicate');
    propGrad = sqrt(propGradX.^2 + propGradY.^2);
    propGrad = normalization(propGrad, 'default', 1, 0);
    propGrad(propGrad >= 0.1) = 0.1;
    % Inverse Instance and Gradient
%     alpha1 = 1.2 * (1 - (propGrad.*(1 - insgrad)));
%     alpha0 = 17*(1 - (propGrad.*(1 - insgrad)));
    % Instance and Gradient
    alpha1 = 1.2 * (1 - propGrad).*(insgrad);
    alpha0 = 17 * (1 - propGrad).*(insgrad);
    % Instance Only
%     alpha1 = 1.2*insgrad;
%     alpha0 = 17*insgrad;
    % Gradient only
%     alpha1 = 1.2 * (1 - propGrad);
%     alpha0 = 17 * (1 - propGrad);
    
    alpha1(alpha1<=0) = 0.000001;
    alpha0(alpha0<=0) = 0.000001;
    
    alpha1old = 1.2;
    alpha0old = 17.0;
    figure, imshow(alpha1);
    pause();
    figure
    
%% TGV L2
    %calculate weight
    weights = zeros(M,N);
    weights(prop > 0) = 1;

    % normalize input depth map
    %d_min = 0.0;
    d_min = min(prop(prop>0));
    d_max = 40.0;%max(ours(ours>0));
    %ours_norm(ours_norm > d_max) = 0;

    min_ = zeros(M, N);
    min_(:,:) = d_min;
    ours_norm = min_./prop;
    ours_norm(ours_norm < 0) = 0;
    ours_norm(ours_norm > 1) = 0;
    ours_norm(prop == 0) = 0;

    % tgv l2
    timestep_lambda = 1;
    
%     tgv_alpha = [17 1.2];
    tensor_ab = [9 0.85];
    lambda_tgvl2 = 5.0;
    factor = 1;

    % iteration number
    maxits = maxIteration;
    disp(' ---- ');

    check = round(maxits/100);
    
    upsampling_result_norm = upsamplingTensorTGVL2_iros2020(ours_norm, ours_norm, ...
        weights.*lambda_tgvl2, gray, tensor_ab, alpha0, alpha1, timestep_lambda, maxits, ...
        check, 0.1, 1);
%     upsampling_result_norm = upsamplingTensorTGVL2(ours_norm, ours_norm, ...
%         weights.*lambda_tgvl2, gray, tensor_ab, [alpha0old alpha1old], timestep_lambda, maxits, ...
%         check, 0.1, 1);

    upsampling_result = min_./upsampling_result_norm;
    upsampling_result(upsampling_result_norm==0) = 0;
    resultOpt = upsampling_result;