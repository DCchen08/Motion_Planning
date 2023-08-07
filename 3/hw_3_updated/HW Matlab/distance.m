function dist = distance(x_new,x_near_poiont)
   delta_x = x_new(1) - x_near_poiont(1);
   delta_y = x_new(2) - x_near_poiont(2);
   dist = sqrt(delta_x^2 + delta_y^2);
end