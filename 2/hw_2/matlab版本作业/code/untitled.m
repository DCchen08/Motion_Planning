clear all
clc
close all

map = [1 1
1	2
1	3
2	3
2	4
2	9
3	1
3	3
4	4
5	3
5	6
5	10
6	2
7	1
7	2
7	3
7	5
7	9
7	10
8	3
8	4
8	8
10	4
10	9
9	9];

MAX_X = 10;
MAX_Y = 10;

size_map = size(map,1);
X_offset = 0;
Y_offset = 0;

MAP=2*(ones(MAX_X,MAX_Y));

xval=floor(map(size_map, 1)) + X_offset;
yval=floor(map(size_map, 2)) + Y_offset;
xTarget=xval;
yTarget=yval;
MAP(xval,yval)=0;

for i = 2: size_map-1
    xval=floor(map(i, 1)) + X_offset;
    yval=floor(map(i, 2)) + Y_offset;
    MAP(xval,yval)=-1;
end 

xval=floor(map(1, 1)) + X_offset;
yval=floor(map(1, 2)) + Y_offset;
xStart=xval;
yStart=yval;
MAP(xval,yval)=1;

OPEN = [];
CLOSED = [];

k=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i;
            CLOSED(k,2)=j;
            k=k+1;
        end
    end
end

CLOSED_COUNT=size(CLOSED,1);
%set the starting node as the first node
xNode=xval;
yNode=yval;
OPEN_COUNT=1;
goal_distance=distance(xNode,yNode,xTarget,yTarget);
path_cost=0;
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
OPEN(OPEN_COUNT,1)=1;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;

Path = [xStart,yStart];
Path_Cnt = 1;

STORRING_OPEN = [];
STORRING_OPEN_CNT = 1;


cnt_run = 1;
while(~isempty(OPEN)) %you have to dicide the Conditions for while loop exit finish the while loop
    i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget); % Check min i line in OPEN List
    if i_min == -1
        fprintf("No Road Available\n")
        return
    end

    tmp_arrary_min_f = OPEN(i_min,:); % The array can find the min f list

    STORRING_OPEN(STORRING_OPEN_CNT,:) = tmp_arrary_min_f;
    STORRING_OPEN_CNT = STORRING_OPEN_CNT + 1;

    OPEN(i_min,:) = []; % delete the row already go through

        Path(Path_Cnt,:) = [tmp_arrary_min_f(2),tmp_arrary_min_f(3)];     %% Storing the Path
        Path_Cnt = Path_Cnt + 1;
    
        CLOSED_COUNT=CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,1) = tmp_arrary_min_f(2);                      %% put the point already go into the CLOSED LIST
        CLOSED(CLOSED_COUNT,2) = tmp_arrary_min_f(3);                      %% put the point already go into the CLOSED LIST
        CLOSED_COUNT=CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,1) = tmp_arrary_min_f(4);
        CLOSED(CLOSED_COUNT,2) = tmp_arrary_min_f(5);

    

    tmp_exp_array = expand_array(tmp_arrary_min_f(2),tmp_arrary_min_f(3),tmp_arrary_min_f(7),xTarget,yTarget,CLOSED,MAX_X,MAX_Y); % Expand with min_f 
    % Insert      ||from Node x|| from Node y || h(n) || g(n) || f(n)
    % MIN_F Array ||to Node x  || to Node y |  | h(n) || g(n) || f(n)

    for idex_exp_arr = 1:1:size(tmp_exp_array,1)
            tmp_arr = tmp_exp_array(idex_exp_arr,:);
            tmp_path = [tmp_arr(1),tmp_arr(2)];
            if any(ismember(tmp_path,CLOSED,"rows"))
                fprintf("skip the path already in closed n")
                continue;
            end
            
            tmp_open = insert_open(tmp_arr(1), ...  % New X Node   %% to Node x,y
                tmp_arr(2), ...                     % New Y Node             
                tmp_arrary_min_f(2), ...            % Parent X Node                               
                tmp_arrary_min_f(3), ...            % Parent Y Node  
                tmp_exp_array(idex_exp_arr,3), ...  % Estimate H                                                                  
                tmp_exp_array(idex_exp_arr,4), ...  % Actual Cost G
                tmp_exp_array(idex_exp_arr,5));     % Heutic f value 
            
            % Check if the OPEN NODE is already in OPEN List
            tmp_open_node = [tmp_open(2),tmp_open(3)];
     
            tmp_all_open_node = [OPEN(:,2), OPEN(:,3)];
            % tmp_open_node
            % tmp_all_open_node
            % g_expand = tmp_arr(4)
            % g_current_in_list = tmp_open(7)
            if (~any(ismember(tmp_open_node,tmp_all_open_node,'rows')))
                fprintf("find same member not in list\n")
                OPEN_COUNT = OPEN_COUNT + 1;
                OPEN(OPEN_COUNT,:) = tmp_open;
                
            elseif (tmp_arr(4) >= tmp_open(7))
                fprintf("Skip the point already in list\n")
                continue;
            end

    end
end




%%
clear
clc
close all

% Example big list with many rows
bigList = [1 2 3; 4 5 6; 7 8 9; 10 11 12; 13 14 15];

% Example list with one row
rowWithOneRow = [7 8 9];

% Use ismember to find the location of the row with one row in the big list
location = ismember(bigList, rowWithOneRow, 'rows');

% Find the row index where the row with one row is located
[rowIndex, ~] = find(location);

