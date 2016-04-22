function main(serPort, mode)
    %init constants
    global floor_level cam_fov robot_radius cam_depth_range_ratio cam_depth_img_width cam_depth_img_center backgrnd
    cam_fov = 74;                            %degrees
    robot_radius = 0.17;                     %meters
    cam_depth_range_ratio = 0.80 / 32000.00; %meters / units
    cam_depth_img_width = 320;               %pixels
    cam_depth_img_center = cam_depth_img_width / 2;

    % Initialize Camera
    CameraHandle  = pxcOpenCamera();
    if(CameraHandle ==0)
        error('no valid camera handle');
    end

    % Initialize Color Tracking
    pxcAcquireFrame(CameraHandle);
    I = pxcColorImage(CameraHandle); I=permute(I([3,2,1],:,:),[3 2 1]);
    pxcReleaseFrame(CameraHandle);
    figure(1);
    imshow(I);
    [col, row] = getpts(1);
    img_double = im2double(I);
    rgb = impixel(img_double, col, row); 
    
    %Init Background Detection
    [backgrnd, ~] = get_camera_image(CameraHandle);
    floor_level = detect_background(backgrnd);
    %h = figure;
    h2=imshow(backgrnd,[200 750]); colormap('jet');
    set(h2,'CDATA',backgrnd);
    %drawnow; 
    input('Press any key and enter to start navigation');
    %figure;
    
    if mode == 0  %camera only mode
        while (1)
            [D, I] = get_camera_image(CameraHandle);
            if identify_obstacle(D)
                display('obstacle detected');
            end
            [~, centroid_x, ~, ~] = color_vision(rgb, I);
            display (centroid_x);
            pause(1);
        end
    elseif mode == 1 % robot navigation mode
        center_on_destination(serPort, CameraHandle, rgb, 0);
        while(1)
            [D, I] = get_camera_image(CameraHandle);
            pause(0.05);
            if identify_obstacle(D)
                if check_destination_arrival(D, I)
                    break;
                end
                SetFwdVelRadiusRoomba(serPort, 0, 0);
                AngleSensorRoomba(serPort);
                display 'Obstacle Identified'
                turn_state = avoid_obstacle(serPort, D);
                theta = 0;
                while identify_obstacle(D)
                    [D, ~] = get_camera_image(CameraHandle);
                    if strcmp(turn_state,'left')
                        turn_left(serPort)
                    else
                        turn_right(serPort)
                    end
                    theta = theta + AngleSensorRoomba(serPort);
                    pause(0.1);
                end
                degree = normalize_radians(theta);
                move_forward_by_distance(serPort, .05, CameraHandle); 
                SetFwdVelRadiusRoomba(serPort, 0, 0);
                pause(1);
                center_on_destination(serPort, CameraHandle, rgb, degree);
            else
                display 'Moving towards destination'
                SetFwdVelRadiusRoomba(serPort, 0.05, inf);
            end
            pause(0.1);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Destination Tracking Code%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function arrived=check_destination_arrival(D, I, rgb)
    global backgrnd
    [~, centroid_x, centroid_y, ~] = color_vision(rgb, I);
    detect_params = detect_object(D, backgrnd);
    
    near_top = max([detect_params.extrema.Extrema(1,2) detect_params.extrema.Extrema(2,2)]);
    near_bottom = min([detect_params.extrema.Extrema(6,2) detect_params.extrema.Extrema(5,2)]);
    near_left = min([detect_params.extrema.Extrema(8,1) detect_params.extrema.Extrema(7,1)]);
    near_right = max([detect_params.extrema.Extrema(4,1) detect_params.extrema.Extrema(3,1));
    
    % color image is double size of depth image, convert coordinates
    centroid_x = double(centroid_x) / 2.00;
    centroid_y = double(centroid_y) / 2.00;
    
    arrived=0;
    if centroid_x > near_left && centroid_x < near_right
        if centroid_y > near_bottom && centroid_y < near_top
            arrived=1;
        end
    end
end

function center_on_destination(serPort, CameraHandle, rgb, degree)
    centered = 0;
    turning = 0;
    while (~centered)
        [~, I] = get_camera_image(CameraHandle);
        [~, centroid_x, ~, ~] = color_vision(rgb, I);
        pause(0.05);
        if centroid_x == 0 
            if turning == 1
                continue;
            elseif degree > 0
                display('Centroid lost. Turning right');
                turn_right(serPort)
            else
                display('Centroid lost, Turning Left');
                turn_left(serPort);
            end
        elseif centroid_x < 250
            display('Centroid found, turning left');
            turn_left(serPort);
        elseif centroid_x > 390
            display('Centroid found, turning right');
            turn_right(serPort);
        else
            SetFwdVelRadiusRoomba(serPort, 0, 0);
            centered = 1;
        end
        turning = 1;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Obstacle Detection/Avoidance Code%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obstacle_identified =identify_obstacle(Camera_Depth_Info)
    global cam_depth_img_width robot_radius backgrnd floor_level
    detect_params = detect_object(Camera_Depth_Info, backgrnd, floor_level);
    obstacle_identified = 0;
    if (~isempty(fieldnames(detect_params)))
        threshold = cam_depth_img_width * robot_radius * 2 / 1.05;
        near_left = detect_params.extrema.Extrema(8,1);
        near_right = detect_params.extrema.Extrema(4,1);
        if ~((near_right < threshold) || (near_left > cam_depth_img_width - threshold))
            obstacle_identified = 1;
        end
    end
end

function turn_state=avoid_obstacle(serPort, D)
    global backgrnd cam_depth_range_ratio cam_fov cam_depth_img_center floor_level
    detect_params = detect_object(D, backgrnd, floor_level);
    
    % calculate obstacle geometry
    median = detect_params.median;
    depth = double(D(median(1),median(2))) * cam_depth_range_ratio;
    near_left = detect_params.extrema.Extrema(8,1);
    near_right = detect_params.extrema.Extrema(4,1);
    angle_left = (cam_depth_img_center - near_left) / cam_depth_img_center * cam_fov / 2.00;
    angle_right = (near_right - cam_depth_img_center) / cam_depth_img_center * cam_fov / 2.00;
    display(cam_depth_img_center - near_left)
    display(near_right - cam_depth_img_center)
    input('Press any key to continue');
    if cam_depth_img_center - near_left < near_right - cam_depth_img_center
        turn_state = 'left';
    else
        turn_state = 'right';
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%Roomba Movement Code%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function turn_left(serPort)
    SetFwdVelRadiusRoomba(serPort, 0.05, 0.01);
end

function turn_right(serPort)
    SetFwdVelRadiusRoomba(serPort, 0.05, -0.01);
end

function move_forward_by_distance(serPort, distance, CameraHandle)
    DistanceSensorRoomba(serPort);
    travelled = 0;
    while travelled < distance
        get_camera_image(CameraHandle);
        SetFwdVelRadiusRoomba(serPort,0.05, inf);
        travelled = travelled - DistanceSensorRoomba(serPort);
        pause(0.1);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%Math Utility Functions%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function degree = normalize_radians(rad)
    degree = rad / pi * 180;
    while degree <= -180
        degree = degree + 180;
    end
    while degree > 180
        degree = degree - 180;
    end
end
