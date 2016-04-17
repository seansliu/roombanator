function main(serPort)
    center_on_destination(serPort);
    while(1)
        if identify_obstacle()
            turn_state = avoid_obstacle(serPort);
            reacquire_destination(serPort, turn_state);
        else
            move_foward(serPort);
        end
    end
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


function obstacle_identified=identify_obstacle()
    near_left, near_right = get_nearest_obstacle();
    if near_left < center && near_right > center
        obstacle_identified = 1;
    else
        obstacle_identified = 0;
    end
end


function turn_state=avoid_obstacle(serPort)
    nearest_left, nearest_right = get_nearest_obstacle();
    if (center - nearest_left) > (nearest_right - center)
        while not (nearest_right.is_center())
            turn_state = right;
            turn_right(serPort);
        end
    else
        while not (nearest_left.is_center())
            turn_state = left;
            turn_left(serPort);
        end
    end
    move_forward(1.0);
end

function turn_left(serPort)

end

function turn_right(serPort)
end

function move_forward(serPort)
end
