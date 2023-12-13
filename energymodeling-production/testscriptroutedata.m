%given: an array of coordinates provided by a mission leader specifying
%locations the rover needs to travel to. 
%Desired; 
%%1) Segmentation of these coordinates into evenly spaced waypoints
%according to required resolution, assuming a linear path between two
%adjecent way points. 
%INPUT: Raw coordinate array with randomly spaced destinations
%OUTPUT: a) coordinates expressed in decimal latitude and longitude values
%           (these are likely the easiest to work within simulink)
%        b) OPTIONAL: the cardinal direction between two waypoints. If extended to
%           3D force balance the slope relative to the vehicles orientation
%           becomes a factor that might introduce extra slippage along the
%           transverse directions, in addition to this if solar charging is
%           incorporated this can be used to identify zenith angles of each
%           solar panel. Finally, this can be used for comparison of the
%           wind direction to the oirientation of the vehicle and may be
%           used to dynamically adjust the vehicle frontal area in aerodrag
%           equation.
%Required manipulations to INPUT: 
%        a) conversion function of general coordinate format to decimal
%        coordinate if needed.
%        b) calculation of distance between coordinates and creation of new
%        coordinates in between two adjacent waypoints in accordance with
%        required separation distance DONE (ChAd GrEeNe)

%2) Association of distance between coordinates with travelled (along surface) distance
%according to the given driving profile
%INPUT: coordinate array from previous step

%3) Associaten of these coordinates with elevation values obtained from
%REMA, which can be converted to overall grade and input into the drag
%force equation for grade
%4) Association of coordinates to (historically averaged) weather data
%provided from Shashwat's suggested model, starting with just wind and temperature.
%5) Association of coordinates (likely grouped into domains) to expected
%values for terrain characteristics such as rolling friction coefficients
