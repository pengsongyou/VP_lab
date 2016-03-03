close all;
clear;
clc;

im = imread('./images/chessboard04.png'); 

im_orig = im;
if size(im,3)>1 
    im=rgb2gray(im); 
end
% Derivative masks
  dx = [-1 0 1;
        -1 0 1;
        -1 0 1];
  dy = dx';
% Image derivatives
Ix = conv2(double(im), dx, 'same'); Iy = conv2(double(im), dy, 'same');
sigma=2;
% Generate Gaussian filter of size 9x9 and std. dev. sigma. 
g = fspecial('gaussian',9, sigma);
% Smoothed squared image derivatives 
Ix2 = conv2(Ix.^2, g, 'same');
Iy2 = conv2(Iy.^2, g, 'same');
Ixy = conv2(Ix.*Iy, g, 'same');

%% Part 1 & 2

% Compute M matrix

% im = imresize(im,[size(im,1)/3,size(im,2)]);
[height, width] = size(im);
E = zeros(height, width);
R = zeros(height, width);
k = 0.04;
tic
for i = 2 : height - 1
    for j = 2 : width - 1
        Ix2_sum = sum(sum(Ix2(i - 1 : i + 1, j - 1 : j + 1)));
        Iy2_sum = sum(sum(Iy2(i - 1 : i + 1, j - 1 : j + 1)));
        Ixy_sum = sum(sum(Ixy(i - 1 : i + 1, j - 1 : j + 1)));
        M = [Ix2_sum, Ixy_sum; Ixy_sum, Iy2_sum];
        det_M = M(1,1) * M(2,2) - M(1,2) * M(2,1);
        trace_M = M(1,1) + M(2,2);
        R(i,j) = det_M - k * (trace_M)^2;
        e = eig(M);
        E(i,j) = e(1);
    end
end
toc
imshow(mat2gray(E));

%% Part 3 & 4

% structure initialization
num_max = 81;
features(num_max).p_x = 0;
features(num_max).p_y = 0;

[val,idx] = sort(E(:),'descend');
[I,J] = ind2sub([height, width],idx);

i = 0;% for loop of all pixels
f = 0;% for loop of feature points


D = E;


while f < num_max
    i = i + 1;
    if D(I(i),J(i)) == 0
        continue    
    else
        f = f + 1;
        features(f).p_x = I(i);
        features(f).p_y = J(i);     
        for ii = -5:5
            if (I(i) + ii) <= 0 || (I(i) + ii) > height
                continue;
            end
            for jj = -5:5
                if (J(i) + jj) <= 0 || (J(i) + jj) > width || (ii == 0 && jj == 0)
                    continue;
                end
                D(I(i)+ii, J(i)+jj) = 0;
            end
        end
               
    end
end

% plot
figure; imshow(im_orig); hold on;
% figure;imshow(mat2gray(E));hold on;
for i=1:size(features,2),
plot(features(i).p_y, features(i).p_x, 'r+');
end

% Part 5
i = 0;
% subp = zeros(num_max,2);
while i < num_max
    i = i + 1;
    x = features(i).p_x;
    y = features(i).p_y;
    A = [];
    b = [];
    for ii = [-1 0 1]
        for jj = [-1 0 1]
            A = [A;(x+ii)^2, (x+ii)*(y+jj), (y+jj)^2,  x+ii, y+jj, 1]; 
            b = [b;E(x+ii,y+jj)];
        end
    end
    p = A\b; % Least mean square to find all the unknown parameters

subp = [2*p(1) p(2);p(2) 2*p(3)]\[-p(4);-p(5)];
if abs(subp(1) - x) > 0.5 || abs(subp(2)-y) > 0.5
%     plot(subp(2), subp(1), 'b+');
    subp(1) = x;
    subp(2) = y;
end
plot(subp(2), subp(1), 'g+');
end