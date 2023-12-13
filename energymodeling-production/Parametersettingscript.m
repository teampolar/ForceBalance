%% Control area for model input parameters
clear
clc
close all

T_start = 0.0;          %Simulation start time
T_stop = 114612;        %Simulation end time
T_step = 0.5;           %Simulation timestep in seconds
Spatial_res= 10;        %Spatial resolution of route in meters

Crr = 0.1; 	            %Rolling friction coefficent        // Should become a column in Route array but am waiting on a good approach for calculating it given the terrain
Mass = 800;             %Vehicle total mass                 // has to be increment
g = 9.81;               %gravitational acceleration
%grade = 1;              %road grade                         // Should become a column in Route array               
%aerodrag related parameters                                // not yet configured correctly now uses vehicle speed as wind speed
% rho = 1.29;           %Air density                        // Should become a column in Route array
% A = 0.6;              %Frontal area w.r.t. wind direction // Might become a column in Route array if bearing and windspeed can be compared and a shape guessed (Really complex likely not worth it) 
% Cd = 0.6;             %Aerodrag coefficient               // Not sure how to best estimate
% v_wind = zeros(height((T_start:T_step:T_stop)'), 2);        %wind speeds array should have at least this many values, likely possible to use simulation time with coordinates in route array together with Shashwats datasets and attach a number to each point along route
temperature = zeros(height((T_start:T_step:T_stop)'), 2);   %same story as wind but for temperature
Battmassfrac = 0.2;         %Fraction of the total mass that is allocated to the battery
BatteryEdensity1 = 150;      %The battery energy density in Wh/kg of battery candidate number 1
BatteryEdensity2 = 200;      %The battery energy density in Wh/kg of battery candidate number 2
BatteryEdensity3 = 400;      %The battery energy density in Wh/kg of battery candidate number 3
BatteryEdensIcecube = 134.4; %The battery energy density in Wh/kg of battery candidate number 3
Packingfactor = 2;           %Reminder for taking this into account later as an offset to the Edensity
%% Routedata
startlat = -68.791257;      %Would rather have a list of intermediate waypoints as well
startlon = 79.149426;
endlat = -73.0333;
endlon = 74.4333;
intermediatewaypoints = [-69.5727 77.9902; -70.1178 76.7158; -70.4145 73.9253; -70.4587 72.2554; -70.6197 74.0786];
waypoints = [startlat startlon; intermediatewaypoints; endlat endlon];
[lati,loni] = pspath([startlat endlat],[startlon endlon],Spatial_res);  %gives interpolated points between the specified waypoints
linpath = [lati ; loni]';        %straightline path between start and end point
% d = pathdistps(lati,loni,'m'); %gives distance between travelled coordinates
% z = bedmachine_interp('surface',lati,loni); %assigns surface elevation to each coordinate
% Z = rema_data(lati,loni) %better elevation model thanks Kaj, similar toolbox available
% plot(d,z)
% xlabel 'distance traveled (m)'
% ylabel 'surface elevation (m)'
% box off
% axis tight
path = [startlat,startlon];
% path(1,1) = startlat;
% path(1,2) = startlon;
for k = 1:height(waypoints)
    if k < height(waypoints)
        b = find(path(1:height(path),1)',1,"last") + 1;
        [interlat,interlon] = pspath([waypoints(k,1) waypoints(k+1,1)],[waypoints(k,2) waypoints(k+1,2)],Spatial_res);
        newsegment = [interlat' interlon']
        path = vertcat(path, newsegment);
    end
end
d = pathdistps(path(:,1)',path(:,2)','m')';                 %calculates the cumulative distance between the interpolated coordinates
z = bedmachine_interp('surface',path(:,1)',path(:,2)')';    %uses the bedmachine dataset to calculate surface elevation


subplot(2,2,1)                                              
plot(d,z)
xlabel 'Full path: distance traveled (m)'
ylabel 'Full path: surface elevation (m)'
grid on
axis tight

Route = [path d z];                             %travelled path as per given coordinates along with travelled distance and surface elevation

%for long paths a slice of the route can be chosen based on starting and
%ending distance, The full transect is always displayed and it is adviced
%to run the simulation once and then check in the graph which sections
%you're interested in.
startdistance = 440000;                 %distance in meters, set to Spatial_res if you want the first entry, this is just a trick since 0 will not return anything (first row index is 1)                                                  
enddistance = Route(end,3);                   %distance in meters, set to Route(end,3) if you want the final entry                            
startslice = startdistance/Spatial_res;                                
endslice = enddistance/Spatial_res;
Routesegment = Route(startslice:endslice,:);
offset_d = Routesegment(1,3); %defines the starting distance in plots for the routesegment, set to zero for comparison with full route
offset_z = Routesegment(1,4); %defines the starting elevation in plots for the routesegment, set to zero for comparison with full route

%makes a plot of elevation versus travelled distance for the segment
subplot(2,2,2)
plot(Routesegment(:,3)-offset_d,Routesegment(:,4)-offset_z) 
xlabel 'pathsegment: distance traveled (m)'
ylabel 'pathsegment: surface elevation (m)'
grid on
axis tight

deltad = [diff(d); 0];                  %used for sanitychecker of generated route later
arraylength = height(Routesegment);     %parameter used in the following section

% OLD VERISON CONTAINING SOME MISTAKES IN FOR-LOOP
% startlat = -68.791257;      %Would rather have a list of intermediate waypoints as well
% startlon = 79.149426;
% endlat = -73.0333;
% endlon = 74.4333;
% intermediatewaypoints = [-69.5727 77.9902; -70.1178 76.7158; -70.4145 73.9253; -70.4587 72.2554; -70.6197 74.0786];
% waypoints = [startlat startlon; intermediatewaypoints; endlat endlon]
% [lati,loni] = pspath([startlat endlat],[startlon endlon],Spatial_res);  %gives interpolated points between the specified waypoints
% linpath = [lati ; loni]';        %straightline path between start and end point
% % d = pathdistps(lati,loni,'m'); %gives distance between travelled coordinates
% % z = bedmachine_interp('surface',lati,loni); %assigns surface elevation to each coordinate
% % Z = rema_data(lati,loni) %better elevation model thanks Kaj, similar toolbox available
% % plot(d,z)
% % xlabel 'distance traveled (m)'
% % ylabel 'surface elevation (m)'
% % box off
% % axis tight
% path = zeros(2,2);
% path(1,1) = startlat;
% path(1,2) = startlon;
% for k = 2:height(intermediatewaypoints)
%     if k < height(intermediatewaypoints)
%         b = find(path(1:height(path),1)',1,"last") + 1;
%         path(b,1) = intermediatewaypoints(k-1,1);
%         path(b,2) = intermediatewaypoints(k-1,2);
%         [interlat,interlon] = pspath([path(b,1) intermediatewaypoints(k,1)],[path(b,2) intermediatewaypoints(k,2)],Spatial_res);
%         newsegment = [interlat' interlon'];
%         path = vertcat(path, newsegment);
%     elseif k == height(intermediatewaypoints)
%         path(b,1) = intermediatewaypoints(k-1,1);
%         path(b,2) = intermediatewaypoints(k-1,2);
%         [interlat,interlon] = pspath([path(b,1) endlat],[path(b,2) endlon],Spatial_res);
%         newsegment = [interlat' interlon'];
%         path = vertcat(path, newsegment);
%     end
% end
% d = pathdistps(path(:,1)',path(:,2)','m')';                 %calculates the cumulative distance between the interpolated coordinates
% z = bedmachine_interp('surface',path(:,1)',path(:,2)')';    %uses the bedmachine dataset to calculate surface elevation
% 
% %creates a plot of surface elevation along the entire path
% subplot(2,2,1)                                              
% plot(d,z)
% xlabel 'Full path: distance traveled (m)'
% ylabel 'Full path: surface elevation (m)'
% grid on
% axis tight
% 
% Route = [path d z];                             %travelled path as per given coordinates along with travelled distance and surface elevation
% %for long paths a slice of the route can be chosen based on starting and ending distance,
% %Doesn't work correctly, would like to make smt with which the first plot
% %can just be clicked in and then the others are generated
% startdistance = 10000;                                                   
% enddistance = 27000;                            
% startslice = startdistance/Spatial_res;                                
% endslice = enddistance/Spatial_res;
% Routesegment = Route(startslice:endslice,:);
% 
% %makes a plot of elevation versus travelled distance for the segment
% subplot(2,2,2)
% plot(Routesegment(:,3)-Routesegment(1,3),Routesegment(:,4)-Routesegment(1,4))   
% xlabel 'pathsegement: distance traveled (m)'
% ylabel 'pathsegement: surface elevation (m)'
% grid on
% axis tight
% 
% arraylength = height(Routesegment);     %parameter used in the following section
%% Velocity profiler

%mechanical constraints used in modelling block 
maxlonacc = 1;                          %maximum longitudinal acceleration in [m/s^2]
maxlondec = 1;                          %maximum longitudinal deceleration in [m/s^2]
maxlonjerk = 1;                         %maximum longitudinal "jerk" in [m/s^3] (derivative of acceleration (serves as smoothener)
maxlatacc = 1;                          %max lateral acceleration in [m/s^2] doesn't serve a purpose for straight paths

% AP S&D: these parameters can be based upon slipage in ice regions (different
% traction systems have different liability of slipping, which is
% essentially a range of accelerations that is tolerable, the maxlonacc can
% be adjusted according to this range after which it can be assumed that no
% slippage occurs)

%inputs used for modelling block
fwdorbwd = ones(arraylength,1);         %driving direction backwards =-1 and forwards is 1
roadcurvatures = zeros(arraylength,1);  %roadcurvature, for now a straight path is assumed
V_start = 0;                            %vehicle initial speed in [m/s]
V_end = 6;                              %vehicle final speed in [m/s]
maxspeed = max(V_start,V_end);          %The maximum allowed speed of the vehicle
lengths = Routesegment(:,3);            %cumulative distance along route, obtained from Route array, in section Routedata

%openExample('driving/VelocityProfileStraightPathExample')
Vmodel = 'VelocityProfileStraightPath';
%open_system(Vmodel)                                        can be used to
%open the model and check parameters of the velocityprofilerblock
out = sim(Vmodel);
t = length(out.tout);
velocities = out.yout.signals(1).values(:,:,t);
times = out.yout.signals(2).values(:,:,t);
distance = velocities.*times;

%plots velocity as function of traveldistance for the segment
subplot(2,2,4);
plot(Routesegment(:,3)-offset_d,velocities)
title('Velocity Profile')
xlabel('distance (m)')
ylabel('Velocities (m/s)')
grid on
axis tight

offset_t = times(1); %defines the starting time in plots for the routesegment, set to zero for comparison with full route
%plots velocity as function of time for the segment
subplot(2,2,3); 
plot(times,velocities)
title('Velocity Profile')
xlabel('Time (s)')
ylabel('Velocities (m/s)')
grid on
axis tight

%Routesegment_out = 
%The subplot below can be used to check whether the generated coordinates
%are not leaving gaps (sanity check for the for loop in section Routedata)
% subplot(2,2,3)
% plot(Route(:,3),deltad) 
% xlabel 'pathsegement: distance traveled (m)'
% ylabel 'pathsegement: surface elevation (m)'
% grid on
% axis tight

Routesegment = [times Routesegment velocities];
%Routesegment now contains all minimal required data for evaluating the force balance along the chosen route, except for aerodrag because windspeed was not yet incorporated;
%column 1 = arrival time at each coordinate
%column 2 = latitude
%column 3 = longitude
%column 4 = travelled distance 
%column 5 = surface elevation
%column 6 = velocity at a given position

moments = Routesegment(:,1)-offset_t;
traveldist = Routesegment(:,4) - offset_d;
segment_z = Routesegment(:,5) -offset_z;
gradient = atan(segment_z./Spatial_res);
Routesegment_out = [moments, Routesegment(:,2), Routesegment(:,3), traveldist, segment_z, gradient velocities];

%Routesegment_out = [moments, Routesegment(:,2), Routesegment(:,3), traveldist, segment_z, gradient, velocities];
%% Drivestrategies (attemps at custom drivestrategies)
% t = T_start:T_step:T_stop;
% v_stratA = 5.*sin(t)+10;
% v_stratB = 10*heaviside(t) ;
% DrivestratA.time = t';
% DrivestratA.signals.values = v_stratA';
% DrivestratB.time = t';
% DrivestratB.signals.values = v_stratB';
% traveldistanceA = cumsum(v_stratA'.*T_step);
% driveprofA = [t',v_stratA',traveldistanceA];
% %Objective: to attach based on the drivestrategy a
% %timestamp to each coordinate in the Route array
% %Issue1: the Route array needs to have a list with timestamps attached to
% %each coordinate
% %Issue 2 the distance between coordinates doesn't necessarily match the distance the
% %vehicle drives within a given timestep
% %Issue 3 the generated drivecycle array length doesn't necessarily match
% %the array length of the route
% % 
% % for j= 1:height(Route)
% %    if j <= height(traveldistanceA)
% %        if traveldistance(j) <= Route(j,3)
% %            v_stratA
% %            Routefinal = vertcat(Route(j,:), 
% % end
% 
% % Make this more user friendly (a UI with at least a scope in which to draw
% % the velocity profiles additionally a map with the routes would also be
% % good to have alongside it)
% 
% 
% 

%% Force balance simulation
sim("massestimate0x2830x29_other.slx")
open("massestimate0x2830x29_other.slx")

