%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 5 Door Tracker
%
% Team number: 12
% Team members: 
%   Nitesh Menon nsm2134
%   Serena Simkus sks2187
%   Sean Liu sl3497
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% main function
function hw5_doortracker_team_12(serPort)

    % constants
    v = 0.2;
    thresh = 0.4;
    hue_range = [.6 .7]; % blue
    sat_range = [.1 .4];

    % clear
    clc;
    [~, ~, ~, ~, ~, ~] = BumpsWheelDropsSensorsRoomba(serPort);
    
    % initialize camera and mask
    img = imread('http://192.168.0.102/img/snapshot.cgi?');
    res = size(img); 
	res = res(1:2);
    center_x = res(2) / 2;
    figure(1);
    imshow(img);
    blue_mask = color_mask(img, hue_range, sat_range);
    imshow(blue_mask);
    
    % initialize state
    % 0: searching for a door
    % 1: door seen
    % 2: door found
    % 3: knock on door and enter
    
    state = 0;

    while (1)
        img = imread('http://192.168.0.102/img/snapshot.cgi?');
        blue_mask = color_mask(img, hue_range, sat_range);
        figure(2);
        imshow(blue_mask);
        [door_found, centroid_x] = find_door(blue_mask, 5000);
        
        switch state
            case 0
                if (centroid_x < (1-thresh) * center_x)
                    SetFwdVelAngVelCreate(serPort, v, 0.5*v);
                elseif (centroid_x > (1+thresh) * center_x)
                    SetFwdVelAngVelCreate(serPort, v, -0.5*v);
                else
                    SetFwdVelAngVelCreate(serPort, 2*v, 0);
                end
                if (door_found)
                    display('found a door!');
                    state = 1;
                end
                
            case 1
                SetFwdVelAngVelCreate(serPort, 0.5*v, 0);
                pause(0.2);
                state = 2;
                
            case 2
                % try to bump into door, if bump, transition to knock
                [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort); 
                if BumpRight
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    turnAngle(serPort, 2*v, -45);
                    state = 3;
                    continue;
                elseif BumpLeft
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    turnAngle(serPort, 2*v, 45);
                    state = 3;
                    continue;
                elseif BumpFront
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    state = 3;
                    continue;
                end
                
                if (~door_found)
                    display('door lost');
                    state = 0;
                    continue;
                end
                
                % move toward door
                if (centroid_x < (1-thresh) * center_x)
                    SetFwdVelAngVelCreate(serPort, 0, v);
                elseif (centroid_x > (1 + thresh)*center_x)
                    SetFwdVelAngVelCreate(serPort, 0, -v);
                else
                    SetFwdVelAngVelCreate(serPort, v, 0);
                end
                
            case 3
                % knock on the door
                SetFwdVelAngVelCreate(serPort, -v, 0);
                pause(.2);
                SetFwdVelAngVelCreate(serPort, v*2, 0);
                pause(.3);
                SetFwdVelAngVelCreate(serPort, -v, 0);
                pause(.2);
                SetFwdVelAngVelCreate(serPort, v*2, 0);
                pause(.3);
                SetFwdVelAngVelCreate(serPort, -v, 0);
                pause(.1);
                
                % beep and wait
                BeepRoomba(serPort);
                pause(.5);
                SetFwdVelAngVelCreate(serPort, 0, 0);
                pause(5);
                
                % go!
                SetFwdVelAngVelCreate(serPort, v, 0);
                pause(3);
                SetFwdVelAngVelCreate(serPort, 0, 0);
                display('we made it!');
                return;
        end
    end
end

%% helper function for creating a binary mask, given an image and color
function mask = color_mask(img, hue_range, sat_range)

    hsv = rgb2hsv(img);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    v = hsv(:,:,3);
    mask = ((h >= hue_range(1)) & (h <= hue_range(2)) & ...
           (s >= sat_range(1)) & (s <= sat_range(2)) & ...
           (v >= 0.1) & (v <= 0.9));
end

%% helper function for finding the a door
function [door_found, centroid_x] = find_door(blue_mask, min_area)

    % initialize values
    door_found = false;
    largest_area = 0;
    largest_index = 0;
   
    % find largest blue region
    regions = regionprops(blue_mask, 'Area', 'Centroid');
    for i = 1:size(regions, 1)
        if (regions(i).Area > largest_area)
            largest_area = regions(i).Area;
            largest_index = i;
        end
    end
    
    % return appropriate values
    if (largest_area > min_area)
        door_found = true;
    end
    if (size(regions, 1) > 0)
        c = regions(largest_index).Centroid;
        centroid_x = c(1);
    else
        centroid_x = -1;
    end
end

