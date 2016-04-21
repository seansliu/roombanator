function main(serPort)
    %init constants
    global cam_fov robot_radius cam_depth_range_ratio cam_depth_img_width cam_depth_img_center backgrnd
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
    [col, row] = getpts(1)
    img_double = im2double(I);
    rgb = impixel(img_double, col, row); 
    
    %Init Background Detection
    h = figure;
    [backgrnd, I] = get_camera_image(CameraHandle);
    h2=imshow(backgrnd,[200 750]); colormap('jet');
    set(h2,'CDATA',backgrnd);
    drawnow; 
    input('Press any key and enter to start navigation');
    figure;

    center_on_destination(serPort, CameraHandle, rgb);
    while(1)
        [D, I] = get_camera_image(CameraHandle);
        if identify_obstacle(D)
            display('obstacle detected.')
            turn_state = avoid_obstacle(serPort,CameraHandle);
            center_on_destination(serPort, CameraHandle, rgb);
        else
            SetFwdVelRadiusRoomba(serPort, 0.05, inf);
        end
        pause(0.1);
    end
end


function [D, I] = get_camera_image(CameraHandle)
        pxcAcquireFrame(CameraHandle);
        D = pxcDepthImage(CameraHandle); D=permute(D,[2 1]);
        I = pxcColorImage(CameraHandle); I=permute(I([3,2,1],:,:),[3 2 1]);
        subplot(1,2,1),h1=imshow(I); 
        subplot(1,2,2),h2=imshow(D,[200 750]); colormap('jet');
        pxcReleaseFrame(CameraHandle);
end


function center_on_destination(serPort, CameraHandle, rgb)
    centered = 0;
    while (~centered)
        [D, I] = get_camera_image(CameraHandle);
        [box, centroid_x, centroid_y, area] = color_vision(rgb, I);
        display (centroid_x);
        pause(0.05)
        if centroid_x == 0
            turn_left(serPort);
        elseif centroid_x < 250
            turn_left(serPort);
        elseif centroid_x > 390
            turn_right(serPort);
        else
            SetFwdVelRadiusRoomba(serPort, 0, 0);
            centered = 1;
        end
    end
end


function obstacle_identified =identify_obstacle(Camera_Depth_Info)
    global cam_depth_img_width robot_radius backgrnd
    detect_params = detect_object(Camera_Depth_Info, backgrnd);
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


function turn_state=avoid_obstacle(serPort, CameraHandle)
    global cam_depth_img_width backgrnd cam_depth_range_ratio cam_fov
    [D, I] = get_camera_image(CameraHandle);
    detect_params = detect_object(D, backgrnd);
    
    % calculate obstacle geometry
    median = detect_params.median;
    depth = double(D(median(1),median(2))) * cam_depth_range_ratio;
    near_left = detect_params.extrema.Extrema(8,1);
    near_right = detect_params.extrema.Extrema(4,1);
    angle_left = (cam_depth_img_center - near_left) / cam_depth_img_center * cam_fov / 2.00
    angle_right = (near_right - cam_depth_img_center) / cam_depth_img_center * cam_fov / 2.00
    
    if near_left > cam_depth_img_width - near_right
        turn_state = 'left';
    else
        turn_state = 'right';
    end
    while identify_obstacle(D)
        if strcmp(turn_state,'left')
            turn_left(serPort)
        else
            turn_right(serPort)
        end
        [D, I] = get_camera_image(CameraHandle);
    end
    move_forward_by_distance(serPort, .1); 
    pause(1);
end

function turn_left(serPort)
    SetFwdVelRadiusRoomba(serPort, 0.05, 0.01);
end

function turn_right(serPort)
    SetFwdVelRadiusRoomba(serPort, 0.05, -0.01);
end

function move_forward_by_distance(serPort, distance)
    DistanceSensorRoomba(serPort);
    travelled = 0;
    while travelled < distance
        SetFwdVelRadiusRoomba(serPort,0.05, inf);
        travelled = travelled - DistanceSensorRoomba(serPort);
        pause(0.1);
    end
end