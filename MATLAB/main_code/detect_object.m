function [params] = detect_object(arr, background_arr)
params = struct([]);
epsilon = 30;
minimum_blob_size = 1000; %in pixels
tolerance = 1000;
[row, column] = size(arr);
object_arr = zeros(row, column);
sum_x = 0;
sum_y = 0;

%create binary array of all pixels closer than tolerance
for r = 1:row
    for c = 1:column
        if background_arr(r,c) >= arr(r,c)- epsilon && background_arr(r,c) <= arr(r,c) + epsilon
            object_arr(r,c) = 0;
        elseif arr(r,c) <= tolerance
            object_arr(r,c) = 1;
        end
    end
end

%find largest blob
blob = zeros(row, column);
CC = bwconncomp(object_arr);
if CC.NumObjects > 0
    numPixels = cellfun(@numel, CC.PixelIdxList);
    [biggest, idx] = max(numPixels);
    blob(CC.PixelIdxList{idx}) = 1;
end

%apply mean filter using a 10 by 10 window
h = fspecial('average', [5,5]);
blob = filter2(h, blob);

%rounding values to make binary matrix
count = 0;
for r = 1:row
    for c = 1:column
        if blob(r,c) == 1
            count = count + 1;
        elseif blob (r,c) >=0.5
            blob(r,c) = 1;
            count = count + 1;
        else
            blob(r,c) = 0;
        end
    end
end

if count < minimum_blob_size
    display('too small');
    blob = zeros(row, column);
end
imshow(blob)

%find median of largest blob
count = sum(blob(:));
for r = 1:row
    for c = 1:column
        if blob(r,c) == 1
            sum_x = sum_x + r;
            sum_y = sum_y + c;
        end
    end
end
median = [uint8(sum_x/count), uint8(sum_y/count)];
if any(blob(:)) == 1
    centroid = regionprops(blob, 'Centroid');
    extrema = regionprops(blob, 'Extrema');
    %store parameters into param struct
    params(1).median = median;
    params(1).blob = blob;
    params(1).centroid = centroid;
    %extrema: [top-left top-right right-top right-bottom 
    %bottom-right bottom-left left-bottom left-top]
    params(1).extrema = extrema;
end

