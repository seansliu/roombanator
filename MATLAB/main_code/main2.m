function main2(serPort)

    % Initialize Camera
    CameraHandle  = pxcOpenCamera();
    if(CameraHandle ==0)
        error('no valid camera handle');
    end
    h = figure;
        pxcAcquireFrame(CameraHandle);
        I = pxcColorImage(CameraHandle); I=permute(I([3,2,1],:,:),[3 2 1]);
        pxcReleaseFrame(CameraHandle);

    figure(1);
    imshow(I);
    [col, row] = getpts(1)
    % initialize from user input
    img_double = im2double(I);
    rgb = impixel(img_double, col, row); 
    
    while (1)
        [D, I] = get_camera_image(CameraHandle);
        [box, centroid_x, centroid_y, area] = color_vision(rgb, I);
        display (centroid_x);
        pause(0.1)
        if centroid_x == 0
                    SetFwdVelRadiusRoomba(serPort, 0, 0);

        elseif centroid_x < 250
        turn_left(serPort);
        elseif centroid_x > 390
        turn_right(serPort);
        else
        SetFwdVelRadiusRoomba(serPort, 0, 0);
        rgb
        col
        row
        end
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


function turn_left(serPort)
    SetFwdVelRadiusRoomba(serPort, 0.05, 0.01);
end

function turn_right(serPort)
    SetFwdVelRadiusRoomba(serPort, 0.05, -0.01);
end

