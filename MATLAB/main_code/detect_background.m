function [ floor_level ] = detect_background( background_arr )
floor_level = 0;
[row, column] = size(background_arr);
row_sum = sum(background_arr,2);
greatest_diff = 0;
floor_level = 0;
for r=5:row-5
    if abs(row_sum(r+1)-row_sum(r)) > greatest_diff
        greatest_diff = abs(row_sum(r+1)-row_sum(r));
        floor_level = r+1;
    end
end
end

