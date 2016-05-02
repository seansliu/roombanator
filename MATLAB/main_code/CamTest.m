function CamTest()
    cam_depth_range_ratio = 0.800 / 1000.000; %meters / units
    cam_depth_img_width = 320; %pixels
    cam_depth_img_center = cam_depth_img_width / 2;
    
    % Initialize Camera
    CameraHandle  = pxcOpenCamera();
    if(CameraHandle ==0)
        error('no valid camera handle');
    end
    
    %Init Background Detection
    %input('Press any key and enter to start background detection');
    h = figure;
    [backgrnd, I] = get_camera_image(CameraHandle);
    h2=imshow(backgrnd,[200 750]); colormap('jet');
    set(h2,'CDATA',backgrnd);
    drawnow; 
    %display('Background Detection Complete');
    input('Press any key and enter to start navigation');
    
    %center_on_destination(serPort, CameraHandle, rgb);
    figure;
    
    
    while (1)
       [D, I] = get_camera_image(CameraHandle);
       detect_params = detect_object(D, backgrnd);
       if (~isempty(fieldnames(detect_params)))

            median = detect_params.median;
            depth = double(D(median(1),median(2))) * cam_depth_range_ratio;
            near_left = detect_params.extrema.Extrema(8,1);
            near_right = detect_params.extrema.Extrema(4,1);
            angle_left = (cam_depth_img_center - near_left) / cam_depth_img_center * 34.00
            angle_right = (near_right - cam_depth_img_center) / cam_depth_img_center * 34.00
       end
       pause(1)
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