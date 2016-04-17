function color_vision(serPort)
    
    % constants
    v = 0.08;
    thresh = 0.2;
    target = [255 0 0];

    CameraHandle  = pxcOpenCamera();
    if(CameraHandle == 0)
        error('no valid camera handle');
    end 

    pxcAcquireFrame(CameraHandle);
    D = pxcDepthImage(CameraHandle); 
    I = pxcColorImage(CameraHandle); 
    D = permute(D,[2 1]);
    I = permute(I([3,2,1],:,:),[3 2 1]);
    pxcReleaseFrame(CameraHandle);
    mask = pixel_mask(I, target);
    regions = regionprops(mask, 'Area', 'BoundingBox', 'Centroid');
    [~, ~, ~, goal_area] = largest_region(regions, mask);

    subplot(1,3,1), h1=imshow(I);
    jmap=jet(256);
    subplot(1,3,2), h2=imshow(ind2rgb(uint8(D/3-100), jmap));
    
    hs=[];
    while(ishandle(h))
        pause(1);

        % Acquire a Camera Frame, and lock
        pxcAcquireFrame(CameraHandle);

        [D,~,UV] = pxcDepthImage(CameraHandle); 
        XYZ = pxcDepthImage2World(CameraHandle,D);
        I = pxcColorImage(CameraHandle);
        
        D  = permute(D,[2 1]);
        UV = permute(UV,[3 2 1]);
        I  = permute(I([3,2,1],:,:),[3 2 1]);
        XYZ = permute(XYZ,[3 2 1]);

        XYZ = XYZ*1000;
        XYZ = WorldCoordinates2ColorImage(XYZ, UV,size(I));
        X = XYZ(:,:,1); Y = XYZ(:,:,2); Z = XYZ(:,:,3);
        Z(Z<200)=200; Z(Z>1000)=1000;
        Z((X<-500)|(X>500)|(Y<-500)|(Y>500))=nan;
        X(X<-500)=-500; X(X>500)= 500;
        Y(Y<-500)=-500; Y(Y>500)= 500;
        [ind,map] = rgb2ind(I,255);
        subplot(1,3,3)
        if (ishandle(hs)), 
            set(hs, 'XData', X); 
            set(hs, 'YData', Y); 
            set(hs, 'ZData', 750-Z);
            set(hs, 'CData', double(ind)/256);
        else
            hs=surf(X, Y, 1000-Z, double(ind)/256, 'EdgeColor', 'None'); 
        end
        colormap(map);
        axis ij
        axis equal
        axis vis3d
        view(-28,49)

        pxcReleaseFrame(CameraHandle);

        set(h1, 'CDATA', I);
        set(h2, 'CDATA', ind2rgb(uint8(D/3-100), jmap));
        drawnow; 

        mask = pixel_mask(I, target);
        regions = regionprops(mask, 'Area', 'BoundingBox', 'Centroid');
        [box, centroid_x, centroid_y, area] = largest_region(regions, 400);
        if (~box)
            display('Target not found. Retrying...');
            SetFwdVelAngVelRoomba(serPort, 0, 0);
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
        end
        
        % check target position
        if (centroid_x < (1-thresh) * center_x)
            display('moving left');
            turn = v;
        elseif (centroid_x > (1+thresh) * center_x)
            display('moving right');
            turn = -v;
        end

        SetFwdVelAngVelRoomba(serPort, speed, turn);
    end

    pxcCloseCamera(CameraHandle);

end

%% helper function for creating a binary mask, given an image and color
function mask = pixel_mask(image, rgb)
    
    % color threshold
    t = .1;
    
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
