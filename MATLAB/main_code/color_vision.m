function [box, centroid_x, centroid_y, area] = color_vision(target, I)  
    % target = rgb pixel
    mask = pixel_mask(I, target);
    regions = regionprops(mask, 'Area', 'BoundingBox', 'Centroid');
    [box, centroid_x, centroid_y, area] = largest_region(regions, 300);
end

%% helper function for creating a binary mask, given an image and color
function mask = pixel_mask(image, rgb)
    % color threshold 
    global show_image
    t = .1;
    
    hsv = rgb2hsv(rgb);
    image_hsv = rgb2hsv(image);
    h = image_hsv(:,:,1);
    s = image_hsv(:,:,2);
    v = image_hsv(:,:,3);
    mask = ((h >= hsv(1)-t) & (h <= hsv(1)+t) & ...
           (s >= hsv(2)-3*t) & (s <= hsv(1)+3*t) & ...
           (v >= 0.1) & (v <= 0.9));
   if show_image
    subplot(2,2,4),h4=imshow(mask);colormap('winter');
   end
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
