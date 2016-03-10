function subp = harris(im, gaussian_size)
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
g = fspecial('gaussian',gaussian_size, sigma);
% Smoothed squared image derivatives 
Ix2 = conv2(Ix.^2, g, 'same');
Iy2 = conv2(Iy.^2, g, 'same');
Ixy = conv2(Ix.*Iy, g, 'same');

%% Compute E and R Matrix

[height, width] = size(im);
E = zeros(height, width);
R = zeros(height, width);
k = 0.04;
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

%% Non-Maximal Suppression

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
%% Sub-pixel Accuracy
i = 0;
subp = zeros(81,2);
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
    if 4*p(1)*p(3) -p(2)*p(2) <= 0
        subp(i,1) = x;
        subp(i,2) = y;
    else
        subp(i,:) = [2*p(1) p(2);p(2) 2*p(3)]\[-p(4);-p(5)];
        if abs(subp(1) - x) > 0.5 || abs(subp(2)-y) > 0.5
            subp(i,1) = x;
            subp(i,2) = y;
        end
    end
end
end