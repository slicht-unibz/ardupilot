wgs84 = wgs84Ellipsoid;
lat0 = 46.4706546247496;
lon0 = 11.3284988701344;
h0 = 450;

leg_length = 40; %m
leg_spacing = 3; %m
number_of_legs = 1  0;
angle_deg = 68.5;
speed = 1.5; %m/s

for i = 1:number_of_legs
    legY(i,:) = [leg_spacing*(i-1);leg_spacing*(i-1)];  %northing meters
    legX(i,:) = [0;leg_length]; % easting meters
    legN(i,:) = sin(angle_deg*pi/180)*legX(i,:) + cos(angle_deg*pi/180)*legY(i,:);
    legE(i,:) = cos(angle_deg*pi/180)*legX(i,:) - sin(angle_deg*pi/180)*legY(i,:);
    [lat(i,1), lon(i,1), h] = ned2geodetic(legN(i,1), legE(i,1), 0, lat0, lon0, h0, wgs84);
    [lat(i,2), lon(i,2), h] = ned2geodetic(legN(i,2), legE(i,2), 0, lat0, lon0, h0, wgs84);
end

mission_file = fopen('matlab_generated_mission.waypoints','w');

fprintf(mission_file,'QGC WPL 110\n');
fprintf(mission_file,['1	1	0	16	0	0	0	0	' num2str(lat0,10) ' ' num2str(lon0,10) '	585.101413	1\n']);
fprintf(mission_file,['1	0	3	178	0.00000000	' num2str(speed,10) ' 0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1\n']);

direction = 1;
for i = 1:number_of_legs
    if (direction==1) 
        fprintf(mission_file,['0 0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	' num2str(lat(i,1),10) ' ' num2str(lon(i,1),10)	' 200.000000	1\n']);
        fprintf(mission_file,['0 0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	' num2str(lat(i,2),10) ' ' num2str(lon(i,2),10)	' 200.000000	1\n']);
        direction = 0;
    else
        fprintf(mission_file,['0 0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	' num2str(lat(i,2),10) ' ' num2str(lon(i,2),10)	' 200.000000	1\n']);
        fprintf(mission_file,['0 0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	' num2str(lat(i,1),10) ' ' num2str(lon(i,1),10)	' 200.000000	1\n']);
        direction = 1;
    end        
end

direction = 0;
for i = 1:number_of_legs
    if (direction==1) 
        fprintf(mission_file,['0 0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	' num2str(lat(i,1),10) ' ' num2str(lon(i,1),10)	' 200.000000	1\n']);
        fprintf(mission_file,['0 0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	' num2str(lat(i,2),10) ' ' num2str(lon(i,2),10)	' 200.000000	1\n']);
        direction = 0;
    else
        fprintf(mission_file,['0 0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	' num2str(lat(i,2),10) ' ' num2str(lon(i,2),10)	' 200.000000	1\n']);
        fprintf(mission_file,['0 0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	' num2str(lat(i,1),10) ' ' num2str(lon(i,1),10)	' 200.000000	1\n']);
        direction = 1;
    end        
end


fclose(mission_file);