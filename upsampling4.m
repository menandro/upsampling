%input image
gray = im2double(rgb2gray(imread('C:\Users\hirat\Desktop\0129\ivdata_faro\image_02\data\im4.png')));
gray = im2double(rgb2gray(imread('C:\Users\hirat\Desktop\0129\ivdata_kitti\image_02\0000000325.png')));

%calculate gradient
gray_ = imfilter(gray, fspecial('gaussian',[3 3], 0.5));
G_x = [-1 1;
       -1 1];
G_y = G_x';
grad_x = imfilter(gray_, G_x, 'replicate');
grad_y = imfilter(gray_, G_y, 'replicate');
grad = sqrt(grad_x.^2 + grad_y.^2);

flow = (double(imread('C:\Users\hirat\Desktop\0129\ivdata_faro\output\im4.png')))/256.0;
flow = (double(imread('C:\Users\hirat\Desktop\0129\ivdata_kitti\output\0000000325.png')))/256.0;

depth = (double(imread('C:\Users\hirat\Desktop\0129\velodyne_raw\image_02\im4.png')))/256.0;
depth = (double(imread('C:\Users\hirat\Desktop\0129\ivdata_kitti\velodyne_raw\image_02\0000000325.png')))/256.0;

velo_raw = (double(imread('C:\Users\hirat\Desktop\0129\ivdata_faro\velodyne_raw\image_02\im4.png')))/256.0;
velo_raw = (double(imread('C:\Users\hirat\Desktop\0129\ivdata_kitti\velodyne_raw\image_02\0000000325.png')))/256.0;

sem = im2double(rgb2gray(imread('C:\Users\hirat\Desktop\0129\ivdata_faro\semantic\sem4.png')));
sem = im2double(rgb2gray(imread('C:\Users\hirat\Desktop\0129\ivdata_kitti\semantic\0000000325.png')));


[M_raw,N_raw] = size(velo_raw);
velo = zeros(M_raw*N_raw,3);
for i = 1:M_raw
    for j = 1:N_raw
        velo((i-1)*N_raw+j,1) = velo_raw(i,j);
        velo((i-1)*N_raw+j,2) = i;
        velo((i-1)*N_raw+j,3) = j;
    end
end
idx = velo(:,1)==0;
velo(idx,:) = [];

count_velo = size(velo,1);
velo_ini = size(velo,1);

mat = zeros(1000000, 3);
velo = [velo;mat];


velo_pos = zeros(size(velo,1),2);
velo_pos(:,1) = velo(:,3);
velo_pos(:,2) = velo(:,2);

    
[M, N] = size(gray);

factor = 1;

ours = zeros(M,N);


r0 = 10000;
sum_ = 0;
count = 0;
count_ = 0;
gs = 0;
gr = 0;
tmp = 0;
gmax_ = 0;

pointMap = zeros(M,N);

%Propagation in the vertical direction
for i = M:-1:1
    count = count + 1;
    disp(count);
    for j = 1:N        
        for k = 1:velo_ini
            if (velo_pos(k,1) >= j-10) && (velo_pos(k,1) < j+10) && (velo_pos(k,2) >= i-10) && (velo_pos(k,2) < i+10)
                r = sqrt((j - velo_pos(k,1))*(j - velo_pos(k,1)) + (i - velo_pos(k,2))*(i - velo_pos(k,2)));
                if r0 > r
                    r0 = r;
                    tmp = k;
                end
            end
        end
        r0 = 10000;
        for k = 1:velo_ini
            if (velo_pos(k,1) >= j-10) && (velo_pos(k,1) < j+10) && (velo_pos(k,2) >= i-10) && (velo_pos(k,2) < i+10)
                
                if round(velo_pos(k,1))>0 && round(velo_pos(k,2))>0 && round(velo_pos(k,1))<=size(grad,2) && round(velo_pos(k,2))<=size(grad,1)
                    
                    if j >= round(velo_pos(k,1))
                        for l = round(velo_pos(k,1)):j
                            if grad(i,l)>gmax_
                                gmax_ = grad(i,l);
                            end
                        end
                        if i >= round(velo_pos(k,2))
                            for l = round(velo_pos(k,2)):i
                                if grad(l,round(velo_pos(k,1)))>gmax_
                                    gmax_ = grad(l,round(velo_pos(k,1)));
                                end
                            end
                        else
                            for l = i:round(velo_pos(k,2))
                                if grad(l,round(velo_pos(k,1)))>gmax_
                                    gmax_ = grad(l,round(velo_pos(k,1)));
                                end
                            end
                        end
                    else
                        for l = j:round(velo_pos(k,1))
                            if grad(i,l)>gmax_
                                gmax_ = grad(i,l);
                            end
                        end
                        if i >= round(velo_pos(k,2))
                            for l = round(velo_pos(k,2)):i
                                if grad(l,round(velo_pos(k,1)))>gmax_
                                    gmax_ = grad(l,round(velo_pos(k,1)));
                                end
                            end
                        else
                            for l = i:round(velo_pos(k,2))
                                if grad(l,round(velo_pos(k,1)))>gmax_
                                    gmax_ = grad(l,round(velo_pos(k,1)));
                                end
                            end
                        end
                    end
                end                
                
                if sem(i,j) > 0.47 && sem(i,j) < 0.48
                    sem(i,j) = sem(283,670); 
                end
                
                gmax_ = gmax_*gmax_;
                
                
                gs = 1.0 / (1.0 + sqrt((j - velo_pos(k,1))*(j - velo_pos(k,1)) + (i - velo_pos(k,2))*(i - velo_pos(k,2))));
                gr = 1.0 / (1.0 + abs(velo(tmp,1) - velo(k,1)));
                
                sum_ = sum_ + (velo(k,1) * gs * gr*(1.0/(gmax_+0.01))) * 1;
                count_ = count_ + (gs * gr*(1.0/(gmax_+0.01))) * 1;
                if sem(i,j) > 0.27 && sem(i,j) < 0.28
                    sum_ = sum_ + (velo(k,1) * gs * gr*(1.0/(gmax_+0.01))) * 10;
                    count_ = count_ + (gs * gr*(1.0/(gmax_+0.01))) * 10;
                end
                
                
                if (velo_pos(k,1) >= 1) && (velo_pos(k,1) < N) && (velo_pos(k,2) >= 1) && (velo_pos(k,2) < M)
                    if sem(i,j) == sem(round(velo_pos(k,2)),round(velo_pos(k,1)))
                        sum_ = sum_ + (velo(k,1)*100*gs*gr*(1.0/(gmax_+0.01))) * 1;
                        count_ = count_ + (100 * gs * gr*(1.0/(gmax_+0.01))) * 1;
                        if sem(i,j) > 0.27 && sem(i,j) < 0.28
                            sum_ = sum_ + (velo(k,1)*10*gs*gr*(1.0/(gmax_+0.01))) * 10;
                            count_ = count_ + (10 * gs * gr*(1.0/(gmax_+0.01))) * 10;
                        end
                    end
                    if sem(i,j) == sem(round(velo_pos(k,2)),round(velo_pos(k,1)))
                        
                        if sem(i,j) > 0.352 && sem(i,j) < 0.353
                            sum_ = sum_ + (velo(k,1)*10*gs*gr*(1.0/(gmax_+0.01))) * 100;
                            count_ = count_ + (10 * gs * gr*(1.0/(gmax_+0.01))) * 100;
                        end
                    end
                end
                gmax_ = 0;
            end
        end
        
        if count_ ~= 0
            pointMap(i,j) = 1;
        end
         sum_ = sum_ * 3;
         count_ = count_ * 3;
        
        
        if i < M-10
            for k = velo_ini + ((M-i-10) * N) + 1 : velo_ini + ((M-i) * N)
                if (velo_pos(k,1) >= j-10) && (velo_pos(k,1) < j+10) && (velo_pos(k,2) >= i-10) && (velo_pos(k,2) < i+10)
                    r = sqrt((j - velo_pos(k,1))*(j - velo_pos(k,1)) + (i - velo_pos(k,2))*(i - velo_pos(k,2)));
                    if r0 > r
                        r0 = r;
                        tmp = k;
                    end
                end
            end
            r0 = 10000;
            for k = velo_ini + ((M-i-10) * N) + 1:velo_ini + ((M-i) * N)
                if (velo_pos(k,1) >= j-10) && (velo_pos(k,1) < j+10) && (velo_pos(k,2) >= i-10) && (velo_pos(k,2) < i+10)
                
                    if round(velo_pos(k,1))>0 && round(velo_pos(k,2))>0 && round(velo_pos(k,1))<=size(grad,2) && round(velo_pos(k,2))<=size(grad,1)
                    
                        if j >= round(velo_pos(k,1))
                            for l = round(velo_pos(k,1)):j
                                if grad(i,l)>gmax_
                                    gmax_ = grad(i,l);
                                end
                            end
                            if i >= round(velo_pos(k,2))
                                for l = round(velo_pos(k,2)):i
                                    if grad(l,round(velo_pos(k,1)))>gmax_
                                        gmax_ = grad(l,round(velo_pos(k,1)));
                                    end
                                end
                            else
                                for l = i:round(velo_pos(k,2))
                                    if grad(l,round(velo_pos(k,1)))>gmax_
                                        gmax_ = grad(l,round(velo_pos(k,1)));
                                    end
                                end
                            end
                        else
                            for l = j:round(velo_pos(k,1))
                                if grad(i,l)>gmax_
                                    gmax_ = grad(i,l);
                                end
                            end
                            if i >= round(velo_pos(k,2))
                                for l = round(velo_pos(k,2)):i
                                    if grad(l,round(velo_pos(k,1)))>gmax_
                                        gmax_ = grad(l,round(velo_pos(k,1)));
                                    end
                                end
                            else
                                for l = i:round(velo_pos(k,2))
                                    if grad(l,round(velo_pos(k,1)))>gmax_
                                        gmax_ = grad(l,round(velo_pos(k,1)));
                                    end
                                end
                            end
                        end
                    end
                    
                    
                    gmax_ = gmax_*gmax_;
                
                
                    gs = 1.0 / (1.0 + sqrt((j - velo_pos(k,1))*(j - velo_pos(k,1)) + (i - velo_pos(k,2))*(i - velo_pos(k,2))));
                    gr = 1.0 / (1.0 + abs(velo(tmp,1) - velo(k,1)));
                    
                    sum_ = sum_ + velo(k,1) * gs * gr*(1.0/(gmax_+0.01));
                    count_ = count_ + gs * gr*(1.0/(gmax_+0.01));
                    if (velo_pos(k,1) >= 1) && (velo_pos(k,1) < N) && (velo_pos(k,2) >= 1) && (velo_pos(k,2) < M)
                        if sem(i,j) == sem(round(velo_pos(k,2)),round(velo_pos(k,1)))
                            sum_ = sum_ + velo(k,1)*100*gs*gr*(1.0/(gmax_+0.01));
                            count_ = count_ + 100 * gs * gr*(1.0/(gmax_+0.01));
                        end
                    end
                    gmax_ = 0;
                end
            end
        end
        
        
%         count_velo = count_velo + 1;
%         velo_pos(count_velo,1) = j;
%         velo_pos(count_velo,2) = i;
%         velo(count_velo,1) = sum_ / count_;
        
        if count_ ~= 0
            count_velo = count_velo + 1;
            velo_pos(count_velo,1) = j;
            velo_pos(count_velo,2) = i;
            velo(count_velo,1) = sum_ / count_;
            ours(i,j) = sum_ / count_;
        else
            %disp('zero');
            count_velo = count_velo + 1;
            velo_pos(count_velo,1) = 10000;
            velo_pos(count_velo,2) = 10000;
            velo(count_velo,1) = 0;
        end
        
     

        
        tmp = 0;
        sum_ = 0;
        count_ = 0;
        
        
    end
end


velo_ini2 = count_velo;
count = 0;

%Propagation in the horizontal direction
for j = N:-1:1
    count = count + 1;
    disp(count);
    for i = M:-1:1
        
        if (sem(i,j) > 0.35 && sem(i,j) < 0.36) || (sem(i,j) > 0.45 && sem(i,j) < 0.46)
        
        for k = 1:velo_ini
            if (velo_pos(k,1) >= j-10) && (velo_pos(k,1) < j+10) && (velo_pos(k,2) >= i-10) && (velo_pos(k,2) < i+10)
                r = sqrt((j - velo_pos(k,1))*(j - velo_pos(k,1)) + (i - velo_pos(k,2))*(i - velo_pos(k,2)));
                if r0 > r
                    r0 = r;
                    tmp = k;
                end
            end
        end
        r0 = 10000;
        for k = 1:velo_ini
            if (velo_pos(k,1) >= j-10) && (velo_pos(k,1) < j+10) && (velo_pos(k,2) >= i-10) && (velo_pos(k,2) < i+10)
                
                if round(velo_pos(k,1))>0 && round(velo_pos(k,2))>0 && round(velo_pos(k,1))<=size(grad,2) && round(velo_pos(k,2))<=size(grad,1)
                    
                    if j >= round(velo_pos(k,1))
                        for l = round(velo_pos(k,1)):j
                            if grad(i,l)>gmax_
                                gmax_ = grad(i,l);
                            end
                        end
                        if i >= round(velo_pos(k,2))
                            for l = round(velo_pos(k,2)):i
                                if grad(l,round(velo_pos(k,1)))>gmax_
                                    gmax_ = grad(l,round(velo_pos(k,1)));
                                end
                            end
                        else
                            for l = i:round(velo_pos(k,2))
                                if grad(l,round(velo_pos(k,1)))>gmax_
                                    gmax_ = grad(l,round(velo_pos(k,1)));
                                end
                            end
                        end
                    else
                        for l = j:round(velo_pos(k,1))
                            if grad(i,l)>gmax_
                                gmax_ = grad(i,l);
                            end
                        end
                        if i >= round(velo_pos(k,2))
                            for l = round(velo_pos(k,2)):i
                                if grad(l,round(velo_pos(k,1)))>gmax_
                                    gmax_ = grad(l,round(velo_pos(k,1)));
                                end
                            end
                        else
                            for l = i:round(velo_pos(k,2))
                                if grad(l,round(velo_pos(k,1)))>gmax_
                                    gmax_ = grad(l,round(velo_pos(k,1)));
                                end
                            end
                        end
                    end
                end
                
                gmax_ = gmax_*gmax_;
                
                if sem(i,j) > 0.47 && sem(i,j) < 0.48
                    sem(i,j) = sem(283,670); 
                end
                
                gs = 1.0 / (1.0 + sqrt((j - velo_pos(k,1))*(j - velo_pos(k,1)) + (i - velo_pos(k,2))*(i - velo_pos(k,2))));
                gr = 1.0 / (1.0 + abs(velo(tmp,1) - velo(k,1)));
                
                sum_ = sum_ + (velo(k,1) * gs * gr*(1.0/(gmax_+0.01))) * 1;
                count_ = count_ + (gs * gr*(1.0/(gmax_+0.01))) * 1;
                if sem(i,j) > 0.27 && sem(i,j) < 0.28
                    sum_ = sum_ + (velo(k,1) * gs * gr*(1.0/(gmax_+0.01))) * 10;
                    count_ = count_ + (gs * gr*(1.0/(gmax_+0.01))) * 10;
                end
                
                
                if (velo_pos(k,1) >= 1) && (velo_pos(k,1) < N) && (velo_pos(k,2) >= 1) && (velo_pos(k,2) < M)
                    if sem(i,j) == sem(round(velo_pos(k,2)),round(velo_pos(k,1)))
                        sum_ = sum_ + (velo(k,1)*100*gs*gr*(1.0/(gmax_+0.01))) * 1;
                        count_ = count_ + (100 * gs * gr*(1.0/(gmax_+0.01))) * 1;
                        if sem(i,j) > 0.27 && sem(i,j) < 0.28
                            sum_ = sum_ + (velo(k,1)*10*gs*gr*(1.0/(gmax_+0.01))) * 10;
                            count_ = count_ + (10 * gs * gr*(1.0/(gmax_+0.01))) * 10;
                        end
                    end
                    if sem(i,j) == sem(round(velo_pos(k,2)),round(velo_pos(k,1)))
                        
                        if sem(i,j) > 0.352 && sem(i,j) < 0.353
                            sum_ = sum_ + (velo(k,1)*10*gs*gr*(1.0/(gmax_+0.01))) * 100;
                            count_ = count_ + (10 * gs * gr*(1.0/(gmax_+0.01))) * 100;
                        end
                    end
                end
                gmax_ = 0;
            end
        end

        
        
        
            for k = velo_ini2:count_velo
                if (velo_pos(k,1) >= j-10) && (velo_pos(k,1) < j+10) && (velo_pos(k,2) >= i-10) && (velo_pos(k,2) < i+10)
                    r = sqrt((j - velo_pos(k,1))*(j - velo_pos(k,1)) + (i - velo_pos(k,2))*(i - velo_pos(k,2)));
                    if r0 > r
                        r0 = r;
                        tmp = k;
                    end
                end
            end
            r0 = 10000;
            for k = velo_ini2:count_velo
                if (velo_pos(k,1) >= j-10) && (velo_pos(k,1) < j+10) && (velo_pos(k,2) >= i-10) && (velo_pos(k,2) < i+10)
                
                    if round(velo_pos(k,1))>0 && round(velo_pos(k,2))>0 && round(velo_pos(k,1))<=size(grad,2) && round(velo_pos(k,2))<=size(grad,1)
                    
                        if j >= round(velo_pos(k,1))
                            for l = round(velo_pos(k,1)):j
                                if grad(i,l)>gmax_
                                    gmax_ = grad(i,l);
                                end
                            end
                            if i >= round(velo_pos(k,2))
                                for l = round(velo_pos(k,2)):i
                                    if grad(l,round(velo_pos(k,1)))>gmax_
                                        gmax_ = grad(l,round(velo_pos(k,1)));
                                    end
                                end
                            else
                                for l = i:round(velo_pos(k,2))
                                    if grad(l,round(velo_pos(k,1)))>gmax_
                                        gmax_ = grad(l,round(velo_pos(k,1)));
                                    end
                                end
                            end
                        else
                            for l = j:round(velo_pos(k,1))
                                if grad(i,l)>gmax_
                                    gmax_ = grad(i,l);
                                end
                            end
                            if i >= round(velo_pos(k,2))
                                for l = round(velo_pos(k,2)):i
                                    if grad(l,round(velo_pos(k,1)))>gmax_
                                        gmax_ = grad(l,round(velo_pos(k,1)));
                                    end
                                end
                            else
                                for l = i:round(velo_pos(k,2))
                                    if grad(l,round(velo_pos(k,1)))>gmax_
                                        gmax_ = grad(l,round(velo_pos(k,1)));
                                    end
                                end
                            end
                        end
                    end                
                    
                    gmax_ = gmax_*gmax_;
                    
                
                    gs = 1.0 / (1.0 + sqrt((j - velo_pos(k,1))*(j - velo_pos(k,1)) + (i - velo_pos(k,2))*(i - velo_pos(k,2))));
                    gr = 1.0 / (1.0 + abs(velo(tmp,1) - velo(k,1)));
                    
                    sum_ = sum_ + velo(k,1) * gs * gr*(1.0/(gmax_+0.01));
                    count_ = count_ + gs * gr*(1.0/(gmax_+0.01));
                    if (velo_pos(k,1) >= 1) && (velo_pos(k,1) < N) && (velo_pos(k,2) >= 1) && (velo_pos(k,2) < M)
                        if sem(i,j) == sem(round(velo_pos(k,2)),round(velo_pos(k,1)))
                            sum_ = sum_ + velo(k,1)*100*gs*gr*(1.0/(gmax_+0.01));
                            count_ = count_ + 100 * gs * gr*(1.0/(gmax_+0.01));
                        end
                    end
                    gmax_ = 0;
                end
            end
        
        
        
        if count_ ~= 0
            count_velo = count_velo + 1;
            velo_pos(count_velo,1) = j;
            velo_pos(count_velo,2) = i;
            velo(count_velo,1) = sum_ / count_;
            ours(i,j) = sum_ / count_;
        else
            count_velo = count_velo + 1;
            velo_pos(count_velo,1) = 10000;
            velo_pos(count_velo,2) = 10000;
            velo(count_velo,1) = 0;
        end
               
        tmp = 0;
        sum_ = 0;
        count_ = 0;
        
        end
    end
end


pointBot = zeros(N,1);
pointBot_ = 0;
for i = 1:N
    for j = M:-1:1
        if pointMap(j,i) == 1
            pointBot_ = j;
        end
    end
    pointBot(i,1) = pointBot_ + 10;
    pointBot_ = 0;
end

result = zeros(M,N);
for j = 1:N
    for i = M:-1:1
        if i > pointBot(j,1)
            result(i,j) = ours(i,j);
        else
            result(i,j) = ours(i,j)*(i/pointBot(j,1))*(i/pointBot(j,1)) + flow(i,j)*(1-(i/pointBot(j,1))*(i/pointBot(j,1)));
        end
    end
end

figure;
imshow(result,[5 30]); title('Result');
colormap(jet);
impixelinfo;
drawnow;


%calculate weight
weights = zeros(M,N);
weights(result > 0) = 1;

% normalize input depth map
%d_min = 0.0;
d_min = min(result(result>0));
d_max = 40.0;%max(ours(ours>0));
    

min_ = zeros(M, N);
min_(:,:) = d_min;
ours_norm = min_./result;
ours_norm(ours_norm < 0) = 0;
ours_norm(ours_norm > 1) = 0;

%% tgv l2

timestep_lambda = 1;

tgv_alpha = [17 1.2];

tensor_ab = [9 0.85];

lambda_tgvl2 = 5.0;

% iteration number
maxits = 200;
disp(' ---- ');

check = round(maxits/100);


upsampling_result_norm = upsamplingTensorTGVL2(ours_norm, ours_norm, ...
weights.*lambda_tgvl2, gray, tensor_ab, tgv_alpha./factor, timestep_lambda, maxits, ...
check, 0.1, 1);


upsampling_result = min_./upsampling_result_norm;
ours_norm(ours_norm > d_max) = 0;

figure;
imshow(upsampling_result,[5 30]); title('Result');
colormap(jet);
impixelinfo;
drawnow;



evalZero = ones(375,1242);
evalZero(depth == 0) = 0;
evalZero(upsampling_result == 0) = 0;

evalMat = sqrt((double(depth)-double(upsampling_result)).^2);
evalMat(evalZero == 0) = 0;

evalValue = sum(sum(evalMat))/nnz(evalZero);

figure;
imshow(evalMat,[0 5]); title('Evaluation');
colormap(jet);
impixelinfo;
drawnow;