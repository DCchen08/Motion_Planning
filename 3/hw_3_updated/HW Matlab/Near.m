function [x_near_index,x_near_point] = Near(x_rand,T)
  min_distance = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2)
  for T_index = 1:size(T.v,2)
      temp_distance = sqrt((x_rand(1)-T.v(T_index).x)^2+(x_rand(2)-T.v(T_index).y)^2)
      if (temp_distance<= min_distance)
          min_distance = temp_distance;
          x_near_point(1) = T.v(T_index).x;
          x_near_point(2) = T.v(T_index).y;
          x_near_index = T_index;
      end
  end
  %x_near = [T.v.x(min_index,:),T.v.y(min_index,:)];
end