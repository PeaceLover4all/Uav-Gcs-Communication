% UAV Data Structure Model M1400 Uav
uav_data = struct();
uav_data.battery_level = 80;  % Percentage
uav_data.gps_latitude = 40.7128;  % Degrees
uav_data.gps_longitude = -74.0060;  % Degrees
uav_data.altitude = 100;  % Meters
uav_data.climb_rate = 7.5;  % m/s
uav_data.cruising_speed = 15.0;  % m/s
uav_data.peak_thrust = 118;  % N
uav_data.vehicle_mass = 2.65;  % Kg
uav_data.recommended_payload_mass = 0.80;  % Kg
uav_data.max_payload_mass = 1.25;  % Kg
uav_data.max_takeoff_weight = 5.55;  % Kg
uav_data.dimensions = 1.03;  % m (rotor shaft distance)
uav_data.flight_time = 45 * 60;  % Seconds (assuming 45 min)
uav_data.operational_temperature = [-10 50];  % Degrees C
uav_data.humidity_max = 90;  % Percent
uav_data.wind_tolerance = 6;  % m/s
uav_data.flight_radius_min = 500;  % m (radio control)
uav_data.flight_radius_waypoints = 40000;  % m (waypoints)
uav_data.ceiling_altitude = 1000;  % m
uav_data.takeoff_altitude = 4000;  % m

% Telemetry Data to Transfer
telemetry_data = [uav_data.battery_level,...
                  uav_data.climb_rate,...
                  uav_data.cruising_speed,...
                  uav_data.gps_latitude,...
                  uav_data.gps_longitude,...
                  uav_data.vehicle_mass,...
                  uav_data.dimensions,...
                  uav_data.max_payload_mass];

% Set up MAVLink Dialect
dialect = mavlinkdialect("common.xml");

% Set up UAV Node
uavNode = mavlinkio(dialect);
uavPort = 14750;
connect(uavNode, "UDP", 'LocalPort', uavPort);

% Set up GCS Node
gcsNode = mavlinkio(dialect);
gcsPort = 14560;
connect(gcsNode, "UDP", 'LocalPort', gcsPort);

% Set up UAV Client
clientStruct = uavNode.LocalClient;
uavClient = mavlinkclient(gcsNode, clientStruct.SystemID, clientStruct.ComponentID);


paramValueSub = mavlinksub(gcsNode, uavClient, 'ATTITUDE', 'BufferSize', 10,...
                            'NewMessageFcn', @(~, msg) disp(msg));

% Request Parameter List
msg = createmsg(dialect, "PARAM_REQUEST_LIST");
msg.Payload.target_system(:) = uavNode.LocalClient.SystemID;
msg.Payload.target_component(:) = uavNode.LocalClient.ComponentID;
sendudpmsg(gcsNode, msg, "127.0.0.1", uavPort);
pause(1);

% Read Single Parameter
msg = createmsg(gcsNode.Dialect, "PARAM_REQUEST_READ");
msg.Payload.param_index(:) = 0;
msg.Payload.target_system(:) = uavNode.LocalClient.SystemID;
msg.Payload.target_component(:) = uavNode.LocalClient.ComponentID;
sendudpmsg(gcsNode, msg, "127.0.0.1", uavPort);
pause(1);

% Write Parameter
msg = createmsg(gcsNode.Dialect, "PARAM_SET");
msg.Payload.param_id(1:12) = "MAX_YAW_RATE";
msg.Payload.param_type(:) = 9;
msg.Payload.param_value(:) = 45;
msg.Payload.target_system(:) = uavNode.LocalClient.SystemID;
msg.Payload.target_component(:) = uavNode.LocalClient.ComponentID;
sendudpmsg(gcsNode, msg, "127.0.0.1", uavPort);
pause(1);

% Send Telemetry Data
for i = 1:10
    % Create a MAVLink message
    msg = createmsg(dialect, "ATTITUDE");

    % Set the telemetry data fields
    msg.Payload.roll(:) = telemetry_data(2);  % climb_rate
    msg.Payload.pitch(:) = telemetry_data(3);  % cruising_speed
    msg.Payload.yaw(:) = telemetry_data(4);  % gps_latitude
    msg.Payload.rollspeed(:) = telemetry_data(5);  % gps_longitude
    msg.Payload.pitchspeed(:) = telemetry_data(6);  % altitude
    msg.Payload.yawspeed(:) = telemetry_data(7);  % cruising_speed
    msg.Payload.time_boot_ms = telemetry_data(8);  % gps_latitude
   

    % Send the message
    sendudpmsg(uavNode, msg, "127.0.0.1", gcsPort);

    % Wait for 1 second
    pause(1);
end



% Load the image
img = imread('image.jpg');

% Display the original image
figure;
imshow(img);

% Load the image
img1 = imread('image1.jpg');

% Display the original image
figure;
imshow(img1);

% Disconnect from Client
disconnect(uavNode);
disconnect(gcsNode);


