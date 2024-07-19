% Set key and message sizes
key_sizes = [128 192 256];
message_sizes = [8 16 64 128];

% Initialize processing time variables
enc_time = zeros(length(key_sizes), length(message_sizes));
dec_time = zeros(length(key_sizes), length(message_sizes));

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

% Loop through key and message sizes
for i = 1:length(key_sizes)
    key_size = key_sizes(i);
    for j = 1:length(message_sizes)
        message_size = message_sizes(j);

% Generate randomkey
        key =randi([0 1], 1, key_size);

        % Convert telemetry data to binary
        telemetry_data_bin = dec2bin(telemetry_data, message_size);

        % Convert binary string to numeric array
        telemetry_data_num = str2double(telemetry_data_bin);

        % Convert double values to integers
        telemetry_data_int = round(telemetry_data_num);

        % Convert integer array to signed integer array
        telemetry_data_int8 = int16(telemetry_data_int);

        % Convert signed integer array to signed integer array of type int8
        telemetry_data_int8_8 = int8(telemetry_data_int8);

        % Convert key to signed integer array of type int8
        key_int8 = int8(key);

        % Start timer for encryption
        tic

        % Perform OTP encryption
        ciphertext = bitxor(telemetry_data_int8_8, key_int8);

        % Stop timer and store encryption time
        enc_time(i, j) = toc;

        % Start timer for decryption
        tic

        % Perform OTP decryption
        plaintext = bitxor(ciphertext, key_int8);

        % Stop timer and store decryption time
        dec_time(i, j) = toc;
    end
end
% Display results
for i = 1:length(key_sizes)
    for j = 1:length(message_sizes)
        fprintf('Key Size: %d bits, Message Size: %d bytes, Encryption Time: %.3f ms, Decryption Time: %.3f ms\n', key_sizes(i), message_sizes(j), enc_time(i, j)*1000, dec_time(i, j)*1000);
    end
end
% Send encrypted telemetry data
for i = 1:10
    % Create a MAVLink message
    msg = createmsg(dialect, "ATTITUDE");

    % Set the encrypted telemetry data fields
    msg.Payload.roll(:) = ciphertext(2);  % climb_rate
    msg.Payload.pitch(:) = ciphertext(3);  % cruising_speed
    msg.Payload.yaw(:) = ciphertext(4);  % gps_latitude
    msg.Payload.rollspeed(:) = ciphertext(5);  % gps_longitude
    msg.Payload.pitchspeed(:) = ciphertext(6);  % altitude
    msg.Payload.yawspeed(:)= ciphertext(7);  % cruising_speed
    msg.Payload.time_boot_ms = ciphertext(8);  % gps_latitude

    % Send the message
    sendudpmsg(uavNode, msg, "127.0.0.1", gcsPort);

    % Wait for 1 second
    pause(1);
end
% Calculate average processing times
avg_encryption_times = mean(enc_time, 3);
avg_decryption_times = mean(dec_time, 3);

% Plot 3D graph of average processing times
figure;
subplot(1, 2, 1);
surf(message_sizes, key_sizes, avg_encryption_times);
xlabel('Message Size [8 16 64 128](bytes)');
ylabel('key sizes [128 192 256] (bits)');
zlabel('Encryption Time (seconds)');
title('OTP Average Encryption Times');

subplot(1, 2, 2);
surf(message_sizes, key_sizes, avg_decryption_times);
xlabel('Message Size [8 16 64 128](bytes)');
ylabel('key sizes [128 192 256] (bits)');
zlabel('Decryption Time (seconds)');
title('OTP Average Decryption Times');


toc
fprintf('Total time required to transfer all telemetry data: %f seconds\n', toc(tic));

% Load the image
img1 = imread('image1.jpg');

% Display the original image
figure;
imshow(img1);
% Load the image
img = imread('image2.jpg');

% Display the original image
figure;
imshow(img);
% Disconnect from Client
disconnect(uavNode);
disconnect(gcsNode);
