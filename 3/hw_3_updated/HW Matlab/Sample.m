function X_rand = Sample(map_x,map_y,x_G,y_G)
    goal = [x_G,y_G];
    if unifrnd(0,1) < 0.5
       X_rand(1)= unifrnd(0,1)* map_x;  
       X_rand(2)= unifrnd(0,1)* map_y;  
    else
       X_rand=goal;
    end

end