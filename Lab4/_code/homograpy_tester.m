% Close figures
close all;

% Set up some stuff
sequences = {'SEQUENCE1', 'SEQUENCE2', 'SEQUENCE3'};
homographies = {'Sequence1Homographies', 'Sequence2Homographies', 'Sequence3Homographies'};
n_images = [16, 9, 18];

% Select number of sequence to check
s = 3;

% Build paths
path = strcat('../_images/', sequences{s}, '/');
homo = strcat(path, 'Sequence', int2str(s), 'Homographies.mat');
img0 = strcat(path, 'Image_00a.png');

% Load homographies
load(homo)
homographyS = eval(homographies{s});

% Read original image
Image_00a = imread(img0);

% Show original image with a fixed point
p_00 = [176 319 1];
figure; imshow(Image_00a); impixelinfo; hold on;
plot(p_00(1), p_00(2), 'gx');

% Plot correspondent point 
for n = 1 : n_images(s)
    % Read image to check
    imgX = strcat(path, 'Image_', num2str(n,'%02.0f'), 'a.png');
    Image_0Xa = imread(imgX);

    % Compute the correspondence of the fixed point
    p_0X = homographyS(n).H * p_00';

    % Show results
    figure; imshow(Image_0Xa); impixelinfo; hold on;
    plot(p_0X(1) / p_0X(3), p_0X(2) / p_0X(3), 'rx');
    %plot(p_0X(1) / p_0X(3), p_0X(2) / p_0X(3), 'rx');
    
    waitforbuttonpress;
    close all;
end