%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 5 Color Tracker
%
% Team number: 12
% Team members: 
%   Nitesh Menon nsm2134
%   Serena Simkus sks2187
%   Sean Liu sl3497
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% main function
function hw5_colortracker_team_12(serPort)
    
    % constants
    v = 0.08;
    thresh = 0.2;
    
    % initialize from camera
    img = imread('http://192.168.0.102/img/snapshot.cgi?');
    res = size(img); 
	res = res(1:2);
    center_x = res(2) / 2;
    
    % get user input
    figure(1);
    imshow(img);
    [col, row] = getpts(1);
    
    % initialize from user input
    img_double = im2double(img);
    rgb = impixel(img_double, col, row); 
    mask = pixel_mask(img, rgb);
    regions = regionprops(mask, 'Area', 'BoundingBox', 'Centroid');
    [~, ~, ~, goal_area] = largest_region(regions, mask);
    
    while (1)
        % let the camera catch up
        pause(1);
        
        % read from camera
        img = imread('http://192.168.0.102/img/snapshot.cgi?');
        figure(1);
        imshow(img);
        
        % make mask
        mask = pixel_mask(img, rgb);
        figure(2);
        imshow(mask);
        
        % find target
        regions = regionprops(mask, 'Area', 'BoundingBox', 'Centroid');
        [box, centroid_x, centroid_y, area] = largest_region(regions, 500);
        
        if (~box)
            display('Target not found. Retrying...');
            SetFwdVelAngVelCreate(serPort, 0, 0);
            continue;
        end
        
        % initialize movement variables
        display('Target found.');
        speed = 0;
        turn = 0;
        
        % check target size
        if (area < (1-thresh) * goal_area)
            display('moving forward');
            speed = v;
        elseif (area > (1+thresh) * goal_area)
            display('moving backward');
            speed = -v;
        else 
            speed = 0;
        end
        
        % check target position
        if (centroid_x < (1-thresh) * center_x)
            display('moving left');
            turn = v;
        elseif (centroid_x > (1+thresh) * center_x)
            display('moving right');
            turn = -v;
        else
            turn = 0;
        end
        
        % do nothing for vertical change, roomba cannot move up/down
        
        % move
        SetFwdVelAngVelCreate(serPort, speed, turn);
        
        rectangle('Position', box, 'EdgeColor', 'r');
        rectangle('Position', [centroid_x,centroid_y,5,5], 'FaceColor', 'g', 'Curvature', 1);
    end

end

%% helper function for creating a binary mask, given an image and color
function mask = pixel_mask(image, rgb)
    
    % color threshold
    t = .05;
    
    hsv = rgb2hsv(rgb);
    image_hsv = rgb2hsv(image);
    h = image_hsv(:,:,1);
    s = image_hsv(:,:,2);
    v = image_hsv(:,:,3);
    mask = ((h >= hsv(1)-t) & (h <= hsv(1)+t) & ...
           (s >= hsv(2)-3*t) & (s <= hsv(1)+3*t) & ...
           (v >= 0.1) & (v <= 0.9));

end

%% helper function for finding the largest region
function [boundingbox, centroid_x, centroid_y, area] = largest_region(regions, min_size)
    
    % initialize values
    largest_area = 0;
    largest_index = 0;
    
    % find largest area
    for i = 1:size(regions, 1)
        if (regions(i).Area > largest_area)
            largest_area = regions(i).Area;
            largest_index = i;
        end
    end
    
    % return appropriate values
    if (largest_area >= min_size)    
        boundingbox = regions(largest_index).BoundingBox;
        c = regions(largest_index).Centroid;
        centroid_x = c(1);
        centroid_y = c(2);
        area = regions(largest_index).Area;
    else
        boundingbox = 0;
        centroid_x = 0;
        centroid_y = 0;
        area = 0;
    end
    
end
