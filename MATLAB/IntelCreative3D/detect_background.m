function [background_arr] = detect_background(arr)
params = struct([]);
maximum = max(arr(:))
[row, column] = size(arr)
background_arr = zeros(row, column);
for r = 1:row
    for c = 1:column
        if arr(r,c) ~= maximum
            background_arr(r,c) = arr(r,c);
        end
    end
end

end

