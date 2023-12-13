%% Routedata
Spatial_res = 10
startlat = -68.791257;      %Would rather have a list of intermediate waypoints as well
startlon = 79.149426;
endlat = -73.0333;
endlon = 74.4333;
intermediatewaypoints = [-69.5727 77.9902; -70.1178 76.7158; -70.4145 73.9253; -70.4587 72.2554; -70.6197 74.0786];
[lati,loni] = pspath([startlat endlat],[startlon endlon],Spatial_res);  %gives interpolated points between the specified waypoints
linpath = [lati ; loni]';        %straightline path between start and end point
% d = pathdistps(lati,loni,'m'); %gives distance between travelled coordinates
% z = bedmachine_interp('surface',lati,loni); %assigns surface elevation to each coordinate
%Z = rema_data(lati,loni) %better elevation model thanks Kaj, similar toolbox available
% plot(d,z)
% xlabel 'distance traveled (m)'
% ylabel 'surface elevation (m)'
% box off
% axis tight
path = zeros(2,2);
path(1,1) = startlat;
path(1,2) = startlon;
for k = 2:height(intermediatewaypoints)
    if k < height(intermediatewaypoints)
        b = find(path(1:height(path),1)',1,"last") + 1;
        path(b,1) = intermediatewaypoints(k-1,1);
        path(b,2) = intermediatewaypoints(k-1,2);
        [interlat,interlon] = pspath([path(b,1) intermediatewaypoints(k,1)],[path(b,2) intermediatewaypoints(k,2)],Spatial_res);
        newsegment = [interlat' interlon'];
        path = vertcat(path, newsegment);
    elseif k == height(intermediatewaypoints)
        path(b,1) = intermediatewaypoints(k-1,1);
        path(b,2) = intermediatewaypoints(k-1,2);
        [interlat,interlon] = pspath([path(b,1) endlat],[path(b,2) endlon],Spatial_res);
        newsegment = [interlat' interlon'];
        path = vertcat(path, newsegment);
    end
end
d = pathdistps(path(:,1)',path(:,2)','m')';
z = bedmachine_interp('surface',path(:,1)',path(:,2)')';
plot(d,z)
xlabel 'distance traveled (m)'
ylabel 'surface elevation (m)'
box off
axis tight
Route = [path d z]; %travelled path as per given coordinates along with travelled distance and surface elevation

startslice = 10000/Spatial_res;
endslice = 27000/Spatial_res;
Routesegment = Route(startslice:endslice,:);
plot(Routesegment(:,3)-Routesegment(1,3),Routesegment(:,4)-Routesegment(1,4))


%% Drivestrategies
t = T_start:T_step:T_stop;
v_stratA = 5.*sin(t)+10;
v_stratB = 10*heaviside(t) ;
DrivestratA.time = t';
DrivestratA.signals.values = v_stratA';
DrivestratB.time = t';
DrivestratB.signals.values = v_stratB';
traveldistanceA = cumsum(v_stratA'.*T_step);
driveprofA = [t',v_stratA',traveldistanceA];

%% velocity profiler

arraylength = height(Routesegment);
lengths = Routesegment(:,3);
fwdorbwd = ones(arraylength,1);
roadcurvatures = zeros(arraylength,1);
V_start = 30;
V_end = 1;
maxspeed = max(V_start,V_end);
%openExample('driving/VelocityProfileStraightPathExample')
model = 'VelocityProfileStraightPath';
%open_system(model)
out = sim(model);
t = length(out.tout);
velocities = out.yout.signals(1).values(:,:,t);
times = out.yout.signals(2).values(:,:,t);
distance = velocities.*times;

subplot(1,2,1);
plot(Routesegment(:,3),velocities)
title('Velocity Profile')
xlabel('distance (m)')
ylabel('Velocities (m/s)')
grid on

subplot(1,2,2); 
plot(times,velocities)
title('Velocity Profile')
xlabel('Time (s)')
ylabel('Velocities (m/s)')
grid on

