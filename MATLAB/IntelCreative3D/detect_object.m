Enter file contents herefunction [params] = detect_object(arr)
params = struct([]);
tolerance = 1000;
[row, column] = size(arr);
object_arr = zeros(row, column);
sum_x = 0;
sum_y = 0;

%create binary array of all pixels closer than tolerance
for r = 1:row
    for c = 1:column
        if arr(r,c) <= tolerance
            objaect_arr(r,c) = 1;
        end
    end
end

%find largest blob
blob = zeros(row, column);
CC = bwconncomp(object_arr);
numPixels = cellfun(@numel, CC.PixelIdxList);
[biggest, idx] = max(numPixels);
blob(CC.PixelIdxList{idx}) = 1;
figure
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
sum_x
sum_y
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

