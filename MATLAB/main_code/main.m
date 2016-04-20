function main(serPort)
    global cam_fov robot_radius cam_depth_range_ratio cam_depth_img_width cam_depth_img_center backgrnd
    %init constants
    cam_fov = 74; %degrees
    robot_radius = 0.17; %meters
    cam_depth_range_ratio = 0.7 / 32000; %meters / units
    cam_depth_img_width = 320; %pixels
    cam_depth_img_center = cam_depth_img_width / 2;

    % Initialize Camera
    CameraHandle  = pxcOpenCamera();
    if(CameraHandle ==0)
        error('no valid camera handle');
    end
    
    input('Press any key and enter to start background detection');
    h = figure;
    [backgrnd, I] = get_camera_image(CameraHandle);
    h2=imshow(backgrnd,[200 750]); colormap('jet');
    set(h2,'CDATA',backgrnd);
    drawnow; 
    %display('Background Detection Complete');
    input('Press any key and enter to start navigation');
    
    %center_on_destination(serPort);
    figure;
    while(1)
        [D, I] = get_camera_image(CameraHandle);
        if identify_obstacle(D)
            %determine obstacle distance, width, calculate turn and
            %distance needed to travel
            display ('obstacle detected');
            %turn_state = avoid_obstacle(serPort,CameraHandle);
            %reacquire_destination(serPort, turn_state);
        else
            display ('no obstacle');
            %SetFwdVelRadiusRoomba(serPort, 0.05, inf);
            %move_foward(serPort);
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

function center_on_destination(serPort)
    centered = 0;
    while centered == 0
        destination_centroid = get_destination();
        if destination_centroid.is_left()
            turn_left(serPort);
        end
        if destination_centroid.is_right()
            turn_right(serPort);
        end
        if destination_centroid.is_center()
            centered = 0;
        end
    end
end


function obstacle_identified =identify_obstacle(Camera_Depth_Info)
    global cam_depth_img_width robot_radius backgrnd
    detect_params = detect_object(Camera_Depth_Info, backgrnd);
    if (~isempty(fieldnames(detect_params)))
        threshold = cam_depth_img_width * robot_radius * 2 / 1.05;
        near_left = detect_params.extrema.Extrema(8,1);
        near_right = detect_params.extrema.Extrema(4,1);
        if (near_right < threshold) || (near_left > cam_depth_img_width - threshold)
            obstacle_identified = 0;
        else
            obstacle_identified = 1;
        end
    else
        obstacle_identified = 0;
    end
end


function turn_state=avoid_obstacle(serPort, CameraHandle)
    %32000 units in depth matrix is roughly 60cm -- roughly 500 units / cm
    %angle we need to turn is roughly: 90 - arccos(x/d), where d is
    %distance to obstacle edge and x is width of robot
    global cam_depth_img_width backgrnd
    [D, I] = get_camera_image(CameraHandle);
    detect_params = detect_object(D, backgrnd);
    near_left = detect_params.extrema.Extrema(8,1);
    near_right = detect_params.extrema.Extrema(4,1);
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
    SetFwdVelRadiusRoomba(serPort, 0, 0);
end

function turn_left(serPort)
    SetFwdVelRadiusRoomba(serPort, 0.05, 0.01);
end

function turn_right(serPort)
    SetFwdVelRadiusRoomba(serPort, 0.05, -0.01);
end

function move_forward(serPort)
    SetFwdVelRadiusRoomba(serPort,0.05, inf);
end