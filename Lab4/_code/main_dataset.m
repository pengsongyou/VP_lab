% Clean Matlab workspace
close all
clear all
clc

% Read image from where to extract patches
im = imread('../_images/Image_base_050.jpg');

% Set the ROI's center of the original image
y0 = 3600;
x0 = 1600;

% Set the size of the image to extract
y1 = 500;
x1 = 750;

% Set an image to sorround the previous one
y2 = 1500;
x2 = 1500;

% Set the levels of noise (respect to 2^8 grayvalues)
noise_id = ['a', 'b', 'c', 'd'];
noise_std = [0, 3, 6, 18];

% For all levels of noise
for i = 1 : size(noise_std, 2)
    % Add noise to the image
    im_noisy = uint8(255 * imnoise(im2double(im), 'gaussian', 0, (noise_std(i) / 256) ^ 2));
 
    % Extract center image and save it at each folder
    patch_1 = im_noisy(y0 - y1 /2 : y0 + y1 / 2 - 1, x0 - x1 / 2 : x0 + x1 / 2 - 1, :);
    imwrite(patch_1, strcat('../_images/SEQUENCE1/Image_00', noise_id(i), '.png'))
    imwrite(patch_1, strcat('../_images/SEQUENCE2/Image_00', noise_id(i), '.png'))
    imwrite(patch_1, strcat('../_images/SEQUENCE3/Image_00', noise_id(i), '.png'))

    % Extract sorrounding image
    patch_2 = im_noisy(y0 - y2 /2 : y0 + y2 / 2 - 1, x0 - x2 / 2 : x0 + x2 / 2 - 1, :);

    % Generate dataset
    h_generator = dataset_generator;
    h_generator.project(patch_1, patch_2, noise_id(i));
    h_generator.scale(patch_1, y1, x1, noise_id(i));
    h_generator.rotate(patch_2, y1, x1, noise_id(i));
end