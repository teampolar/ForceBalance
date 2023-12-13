clear
clc
close all

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
path = zeros(1,1);
path(1,1) = startlat;
path(1,2) = startlon;
for k = 1:height(waypoints)
    if k < height(waypoints)
        b = find(path(1:height(path),1)',1,"last") + 1
        [interlat,interlon] = pspath([waypoints(k,1) waypoints(k+1,1)],[waypoints(k,2) waypoints(k+1,2)],Spatial_res);
        newsegment = [interlat' interlon'];
        path = vertcat(path, newsegment);
    elseif k == height(waypoints)
        [interlat,interlon] = pspath([waypoints(k-1) endlat],[waypoints(k-1) endlon],Spatial_res);
        newsegment = [interlat' interlon'];
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
%for long paths a slice of the route can be chosen based on starting and ending distance,
%Doesn't work correctly, would like to make smt with which the first plot
%can just be clicked in and then the others are generated
startdistance = 15000;                                                   
enddistance = 30000;                            
startslice = startdistance/Spatial_res;                                
endslice = enddistance/Spatial_res;
Routesegment = Route(startslice:endslice,:);

%makes a plot of elevation versus travelled distance for the segment
subplot(2,2,2)
plot(Routesegment(:,3)-Routesegment(1,3),Routesegment(:,4)-Routesegment(1,4))   
xlabel 'pathsegement: distance traveled (m)'
ylabel 'pathsegement: surface elevation (m)'
grid on
axis tight

arraylength = height(Routesegment);     %parameter used in the following section