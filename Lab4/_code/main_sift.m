% Clean Matlab workspace
close all
clear all
clc

% Select sequence to check
s = 3;

% Select threshold [px]
threshold = 2;

% Is the base image always without noise?
base = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  DO NOT MODIFY ANYTHING BELOW THIS LINE                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set up some stuff
sequences = {'SEQUENCE1', 'SEQUENCE2', 'SEQUENCE3'};
homographies = {'Sequence1Homographies', 'Sequence2Homographies', 'Sequence3Homographies'};
n_images = [16, 9, 18];

% Build some paths
path = strcat('../_images/', sequences{s}, '/');
homo = strcat(path, 'Sequence', int2str(s), 'Homographies.mat');

% Load homographies
load(homo);
homographyS = eval(homographies{s});

% Initialise output
accuracy = zeros(4, n_images(s));

% For each level of noise
noise = {'a', 'b', 'c', 'd'};
for n = 1 : length(noise)
    % Read original image
    if (~base || (base && noise{n} == 'a'))
        img0 = strcat(path, 'Image_00', noise{n}, '.png');
        Image_00 = imread(img0);
        
        % Compute SIFT on the original image
        %[F00, D00] = vl_sift(single(im2double(rgb2gray(Image_00))));
        [F00, D00] = vl_sift(single(im2double(rgb2gray(Image_00))), 'windowsize', 1.5);        
    end
    
    % Count the correct matches on each image 
    for i = 1 : n_images(s)
        % Show something on the terminal
        display(sprintf('Computing image %i / %i \n', ((n - 1) * n_images(s) + i), (length(noise) * n_images(s))));
        
        % Read image to check
        imgX = strcat(path, 'Image_', num2str(i,'%02.0f'), noise{n}, '.png');
        Image_0X = imread(imgX);

        % Compute SIFT on the moving image
        %[F0X, D0X] = vl_sift(single(im2double(rgb2gray(Image_0X))));
        [F0X, D0X] = vl_sift(single(im2double(rgb2gray(Image_0X))), 'windowsize', 1.5); 
        
        % Look for the matches in both images
        [matches, ~] = vl_ubcmatch(D00, D0X);

        % Check the SIFT matches
        counter = 0;
        for m = 1 : size(matches, 2)
            % Compute the correspondent point (ground truth)
            p_00 = [F00(1 : 2, matches(1, m)); 1];
            p_0X = homographyS(i).H * p_00;
            p_0X = p_0X / p_0X(3);

            % Check the distance between the ground truth and the SIFT match
            if norm(p_0X - [F0X(1 : 2, matches(2, m)); 1]) <= threshold
               counter = counter + 1; 
            end
        end
            
        % Compute accuracy and store it
        accuracy(n, i) = counter / size(matches, 2);
    end
end

% Prepare axis to plot
it = 1;
switch s
    case 1
        % Define plotting stuff
        x_axis = [-40 : 10 : -10, 10 : 10 : 40];
        x_label = {'Tilt angle [degrees]', 'Pan angle [degrees]'};
        x_lim = [-40 40];
        it = 2;
        intervals = [1 : 8;
                     9 : 16];
                 
        % Arrange information order
        accuracy(:, :) = accuracy(:, [8 7 6 5 1 2 3 4 16 15 14 13 9 10 11 12]);
        
    case 2
        % Define plotting stuff
        x_axis = 110 : 5 : (110 + 5 * 8);
        x_label = {'Scale factor [%]'};
        x_lim = [110 150];
        intervals = [1 : size(x_axis, 2)];
        
    case 3
        % Define plotting stuff
        x_axis = linspace(-45, 45, 18);
        x_label = {'Rotation angle [degrees]'};
        x_lim = [-45 45];
        intervals = [1 : size(x_axis, 2)];
end

% Plot results
for i = 1 : it
    figure;
    plot(x_axis, 100 * accuracy(1, intervals(i, :)), 'k-x'); hold on;
    plot(x_axis, 100 * accuracy(2, intervals(i, :)), 'b-x'); hold on;
    plot(x_axis, 100 * accuracy(3, intervals(i, :)), 'g-x'); hold on;
    plot(x_axis, 100 * accuracy(4, intervals(i, :)), 'r-x'); hold on;
    xlim(x_lim);
    xlabel(x_label(i));
    ylim([60 100]);
    ylabel('Accuracy [%]');
    legend('sigma = 0', 'sigma = 3', 'sigma = 6', 'sigma = 18', 'Location', 'southeast')
end