% Handler of the functions
function h = dataset_generator
    h.project = @project;
    h.rotate = @rotate;
    h.scale = @scale;
end

% Projection function
function project(patch_small, patch_large, noise_id)
    % Set the number of projections to compute
    number_projects = 16;
    d = number_projects / 4;
    
    % Initialise homography output
    Sequence1Homographies(1 : number_projects) = struct('H', []);
    
    % Vertical direction
    PROJECT = generate_projective(patch_small, patch_large, 20, d, 'vertical');
    for i = 1 : d
        Sequence1Homographies(i).H = PROJECT(i).H;
        imwrite(PROJECT(i).I, strcat('../_images/SEQUENCE1/Image_', num2str(i,'%02.0f'), noise_id, '.png'))
    end
    
    PROJECT = generate_projective(patch_small, patch_large, -20, d, 'vertical');
    for i = d + 1: 2 * d
        Sequence1Homographies(i).H = PROJECT(i - d).H;
        imwrite(PROJECT(i - d).I, strcat('../_images/SEQUENCE1/Image_', num2str(i,'%02.0f'), noise_id, '.png'))
    end
    
    % Horizontal direction
    PROJECT = generate_projective(patch_small, patch_large, 20, d, 'horizontal');
    for i = 2 * d + 1 : 3 * d
        Sequence1Homographies(i).H = PROJECT(i - 2 * d).H;
        imwrite(PROJECT(i - 2 * d).I, strcat('../_images/SEQUENCE1/Image_', num2str(i,'%02.0f'), noise_id, '.png'))
    end
    
    PROJECT = generate_projective(patch_small, patch_large, -20, d, 'horizontal');
    for i = 3 * d + 1 : 4 * d
        Sequence1Homographies(i).H = PROJECT(i - 3 * d).H;
        imwrite(PROJECT(i - 3 * d).I, strcat('../_images/SEQUENCE1/Image_', num2str(i,'%02.0f'), noise_id, '.png'))
    end
    
    % Save structure of homographies
    save('../_images/SEQUENCE1/Sequence1Homographies.mat' , 'Sequence1Homographies');
end

% Scaling function
function scale(patch_small, y1, x1, noise_id)
    % Set the scales to compute
    mid_w = x1 / 2;
    mid_h = y1 / 2;
    s = 1.1; % starting scale
    number_scales = 9;

    % Initialise homography output
    Sequence2Homographies(1 : number_scales) = struct('H', []);
    
    for i = 1 : 9
        scale_factor = s + (i - 1) * 0.05;
        w = x1 / scale_factor; 
        h = y1 / scale_factor;
        patch_current = patch_small(round(mid_h - h / 2) : round(mid_h + h / 2), round(mid_w - w / 2) : round(mid_w + w / 2),:);
        patch_zoom = imresize(patch_current,[500,750]);
        imwrite(patch_zoom, strcat('../_images/SEQUENCE2/Image_', num2str(i,'%02.0f'), noise_id, '.png'))
        H = [scale_factor, 0, mid_w - mid_w * scale_factor;
             0, scale_factor, mid_h - mid_h * scale_factor;
             0, 0, 1];
        % Save homography in the structure 
        Sequence2Homographies(i).H = H;
    end
    
    % Save structure of homographies
    save('../_images/SEQUENCE2/Sequence2Homographies.mat','Sequence2Homographies');
end

% Rotation function
function rotate(patch_2, y1, x1, noise_id)
    % Set the rotations to compute
    low_angle = -45;
    high_angle = 45;
    number_rotations = 18;
    angles = linspace(low_angle, high_angle, number_rotations);

    % Initialise homography output
    Sequence3Homographies(1 : number_rotations) = struct('H', []);
    
    % Transform image and get homography
    for i = 1 : size(angles, 2)    
        % Rotate surrounding image
        patch_2_rotated = imrotate(patch_2, angles(1, i));

        % Take the rotated patch with respect to the center
        x0 = size(patch_2_rotated, 2) / 2;
        y0 = size(patch_2_rotated, 1) / 2;
        tmp = patch_2_rotated(y0 - y1 /2 : y0 + y1 / 2 - 1, x0 - x1 / 2 : x0 + x1 / 2 - 1, :);

        % Save the computed image
        imwrite(tmp, strcat('../_images/SEQUENCE3/Image_', num2str(i,'%02.0f'), noise_id, '.png'))

        % Compute homography (rotation with respect to the center)
        theta = angles(1, i);
        x0 = size(tmp, 2) / 2;
        y0 = size(tmp, 1) / 2;
        alpha = cosd(theta);
        beta = sind(theta);
        H = [alpha beta  (1 - alpha) * x0 - beta * y0;
            -beta  alpha beta * x0 + (1 - alpha) * y0;
            0 0 1];  % multiply by [x y 1]

        % Save homography in the structure
        Sequence3Homographies(i).H = H;
    end

    % Save structure of homographies
    save('../_images/SEQUENCE3/Sequence3Homographies.mat','Sequence3Homographies');
end

% Function to perform the projective transformation
function PROJECT = generate_projective(patch_small, patch_large, offset_base, num_im, project_type)
    % Initialize the struct that keeps H and images    
    PROJECT(1 : num_im) = struct('H', [], 'I', []);

    [y1, x1, ~] = size(patch_small);
    [y2, x2, ~] = size(patch_large);

    w_h = 2400;% Window height
    w_w = 2400;% Window width
    I = zeros(w_h, w_w, 3);

    % Put the large image in a larger window. Only use part of patch_large 
    % image in order to get siginificant projective results
    I_h = 920; I_w = 920;
    I(w_h/2 - I_h/2 + 1 : w_h/2 + I_h/2, w_w/2 - I_w/2 + 1 : w_w/2 + I_w/2,1:3) = patch_large(y2/2 - I_h/2 + 1 : y2/2 + I_h/2, x2/2 - I_w/2 + 1 : x2/2 + I_w/2,:);
    
    % Main loop
    for i = 1 : num_im
        h = I_h;
        w = I_w;

        % The camera rotates in a certain direction and this is the offset
        % that the image plane moves
        offset = offset_base * i; 

        % Radius of the circle
        R = sqrt((h / 2) ^ 2 + (w / 2) ^ 2);    

        % Calculate the corresponding corner points in the trapezoid
        switch project_type
            case 'vertical'
                up_height = w_h/2 - h/2 + 1 - offset;
                down_height = w_h/2 + h/2 - offset;
                
                % Half baseline of trapezoid
                half_base = sqrt(R ^ 2 - (h / 2 - offset) ^ 2); 
                down_width_left = round(size(I,2)/2 - half_base);
                down_width_right = round(size(I,2)/2 + half_base);

                % Half topline of trapezoid
                half_top = sqrt(R ^ 2 - (h / 2 + offset) ^ 2); 
                up_width_left = round(size(I,2)/2 - half_top);
                up_width_right = round(size(I,2)/2 + half_top);

                % Corresponding 4 corner points after warping
                p1 = [up_width_left up_height];
                p2 = [up_width_right up_height];
                p3 = [down_width_right down_height];
                p4 = [down_width_left down_height];

            case 'horizontal'
                left_width = w_w/2 - w/2 + 1 + offset;
                right_width = w_w/2 + w/2 + 1 + offset;

                % Half leftline of trapezoid
                half_base_left = sqrt(R ^ 2 - (w / 2 - offset) ^ 2);
                up_left = round(size(I,2)/2 - half_base_left);
                down_left = round(size(I,2)/2 + half_base_left);

                % Half rightline of trapezoid
                half_base_right= sqrt(R ^ 2 - (w / 2 + offset) ^ 2);
                up_right = round(size(I,2)/2 - half_base_right);
                down_right = round(size(I,2)/2 + half_base_right);

                % Corresponding 4 corner points after warping
                p1 = [left_width up_left];
                p2 = [right_width up_right];
                p3 = [right_width down_right];
                p4 = [left_width down_left];
            otherwise
                error('Unknown registration type');
        end


        % Create perspective transformation that warps the original image
        % coordinates to the trapezoid
        movingPoints = [(w_h/2 - h/2 + 1) (w_h/2 - h/2 + 1); (w_h/2 + h/2)  (w_h/2 - h/2 + 1); (w_h/2 + h/2) (w_h/2 + h/2); (w_h/2 - h/2 + 1) (w_h/2 + h/2)];
        fixedPoints = [p1; p2; p3; p4];
        tform = fitgeotrans(movingPoints, fixedPoints, 'Projective');

        % Create a reference coordinate system where the extent is the size
        % of this image
        RA = imref2d([size(I,1) size(I,2)], [1 size(I,2)], [1 size(I,1)]);

        % Wrap the image
        [I_projective, ~] = imwarp(I, tform, 'OutputView', RA);
        T_large = (tform.T)';

        % Get 500*750 size of small patch from the projective large patch
        patch_projective = I_projective(w_h / 2 - y1 / 2 + 1 : w_h / 2 + y1 / 2, w_w / 2 - x1 / 2 + 1: w_w / 2 + x1 / 2,:);
        PROJECT(i).I = uint8(patch_projective);

        % Transform coordinate from 500*750 to patch_large
        t1 = [1 0 375;
              0 1 500;
              0 0 1];
        % Transform from patch_large to bigger window
        t2 = [1 0 450;
              0 1 450
              0 0 1];
        % Transform back from bigger window to patch_large  
        t1_ = [1 0 -375;
              0 1 -500;
              0 0 1];
        % Transform back from patch_large to 500*750 image
        t2_ = [1 0 -450;
               0 1 -450
               0 0 1]; 

        % Build the final projective matrix
        PROJECT(i).H = t1_ * t2_ * T_large * t2 * t1;
    end
end
