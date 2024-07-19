clear all;
close all;

% Define UAV data structure
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

% Generate random keys for RSA
public_key = struct('n', 0, 'e', 0);
private_key = struct('n', 0, 'd', 0);

public_key.n = randi([100, 1000]); % Random value for n (modulus)
public_key.e = randi([3, 100]); % Random value for e (public exponent)

% Calculate d (private exponent) using the Extended Euclidean Algorithm
[~, d, ~] = gcd(public_key.e, euler_phi(public_key.n));
private_key.d = mod(d, euler_phi(public_key.n)); % Ensure d is positive and less than euler_phi(n)

% Telemetry Data to Transfer
telemetry_data = [uav_data.battery_level,...
                  uav_data.climb_rate,...
                  uav_data.cruising_speed,...
                  uav_data.gps_latitude,...
                  uav_data.gps_longitude,...
                  uav_data.vehicle_mass,...
                  uav_data.dimensions,...
                  uav_data.max_payload_mass];

% Initialize arrays for processing times
encryption_times = zeros(1, 10);
decryption_times = zeros(1, 10);

% RSA Encrypt telemetry data
for i = 1:10
    tic
    encrypted_telemetry_data = rsa_encrypt(telemetry_data, public_key);
    encryption_times(i) = toc;
end

% RSA Decrypt telemetry data
for i = 1:10
    tic
    decrypted_telemetry_data = rsa_decrypt(encrypted_telemetry_data, private_key);
    decryption_times(i) = toc;
end

% Display average processing times
fprintf('Average Encryption Time: %f seconds\n', mean(encryption_times));
fprintf('Average Decryption Time: %f seconds\n', mean(decryption_times));

% Set up MAVLink Dialect
dialect = mavlinkdialect("common.xml");

% Set up UAV Node
uavNode = mavlinkio(dialect);
uavPort = 14750;
tic
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

% Send Encrypted Telemetry Data
for i = 1:10
    % Create a MAVLink message
    msg = createmsg(dialect, "ATTITUDE");

    % Set the encrypted telemetry data fields
    msg.Payload.roll(:) = encrypted_telemetry_data(1);  % battery_level
    msg.Payload.pitch(:) = encrypted_telemetry_data(2);  % climb_rate
    msg.Payload.yaw(:) = encrypted_telemetry_data(3);  % cruising_speed
    msg.Payload.rollspeed(:) = encrypted_telemetry_data(4);  % gps_latitude
    msg.Payload.pitchspeed(:) = encrypted_telemetry_data(5);  % gps_longitude
    msg.Payload.yawspeed(:) = encrypted_telemetry_data(6);  % vehicle_mass
    msg.Payload.time_boot_ms = encrypted_telemetry_data(7);  % dimensions

    % Send the message
    sendudpmsg(uavNode, msg, "127.0.0.1", gcsPort);

    % Wait for 1 second
    pause(1);
end

% Disconnect from Client
disconnect(uavNode);
disconnect(gcsNode);


% RSA decryption at GCS
decrypted_telemetry_data = rsa_decrypt(encrypted_telemetry_data, private_key);
toc
fprintf('Total time required to transfer all telemetry data: %f seconds\n', toc(tic));

% Load the image
img2 = imread('image1.jpg');

% Display the original image
figure;
imshow(img2);

% Load the image
img1 = imread('image5.jpg');

% Display the original image
figure;
imshow(img1);

% RSA encryption and decryption functions
function encrypted_message = rsa_encrypt(message, public_key)
    % RSA encryption algorithm implementation
    encrypted_message = mod(message.^public_key.e, public_key.n);
end

function decrypted_message = rsa_decrypt(encrypted_message, private_key)
    % RSA decryption algorithm implementation
    decrypted_message = mod(encrypted_message.^private_key.d, private_key.n);
end

% Euler's totient function (phi)
function phi = euler_phi(n)
    phi = 0;
    for i =1:n
        if iscoprime(i, n)
            phi = phi + 1;
        end
    end
end

% Function to check if two numbers are coprime
function result = iscoprime(a, b)
    result = gcd(a, b) == 1;
end