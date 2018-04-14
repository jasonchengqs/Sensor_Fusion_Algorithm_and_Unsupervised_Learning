% madgwick.m


%% Start of script
function [imu] = madgwick(imu, comboId, f, beta, session)
addpath('quaternion_library');      % include quaternion library
% close all;                          % close all figures
% clear;                              % clear all variables
% clc;                                % clear the command terminal

%% Import and plot sensor data
g = 9.8;
% imu = dataLoad();
% comboId = 1;
% session = 'trial';
names = fieldnames(imu);

for i = 1:length(names)
    curData = imu.(names{i}).data.(strcat('combo_',num2str(comboId)));
    for k = 1:length(curData.(session))
        accX = curData.(session)(k).sensor_frame.accel.X/g;
%             accX = movmean(accX, avg_window);
        accY = curData.(session)(k).sensor_frame.accel.Y/g;
%             accY = movmean(accY, avg_window);
        accZ = curData.(session)(k).sensor_frame.accel.Z/g;
%             accZ = movmean(accZ, avg_window);
        gyroX = curData.(session)(k).sensor_frame.gyro.X;
        gyroY = curData.(session)(k).sensor_frame.gyro.Y;
        gyroZ = curData.(session)(k).sensor_frame.gyro.Z;
        magnetX = curData.(session)(k).sensor_frame.magnet.X/100;
        magnetY = curData.(session)(k).sensor_frame.magnet.Y/100;
        magnetZ = curData.(session)(k).sensor_frame.magnet.Z/100;

        Gyroscope = [gyroZ, gyroY, gyroX];
        Accelerometer = [accZ, accY, accX];
        Magnetometer = [magnetZ, magnetY, magnetX];

        AHRS = MadgwickAHRS('SamplePeriod', 1/f, 'Beta', beta);
        quaternion = zeros(length(gyroX), 4);
        for t = 1:length(gyroX)
            AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
            quaternion(t, :) = AHRS.Quaternion;
        end
        euler = quatern2euler(quaternConj(quaternion));
        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.madgwick_quat.q1 = quaternion(:,1);
        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.madgwick_quat.q2 = quaternion(:,2);
        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.madgwick_quat.q3 = quaternion(:,3);
        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.madgwick_quat.q4 = quaternion(:,4);
        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.madgwick_angle.X = euler(:,3);
        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.madgwick_angle.Y = euler(:,2);
        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.madgwick_angle.Z = euler(:,1);
    end 
end
% time = curData.time - curData.time(1);
% figure('Name', 'Sensor Data');
% axis(1) = subplot(3,1,1);
% hold on;
% plot(time, curData.gyro.X/pi*180, 'r');
% plot(time, curData.gyro.Y/pi*180, 'g');
% plot(time, curData.gyro.Z/pi*180, 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Angular rate (deg/s)');
% title('Gyroscope');
% hold off;
% axis(2) = subplot(3,1,2);
% hold on;
% plot(time, curData.accel.X/9.8, 'r');
% plot(time, curData.accel.Y/9.8, 'g');
% plot(time, curData.accel.Z/9.8, 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Acceleration (g)');
% title('Accelerometer');
% hold off;
% axis(3) = subplot(3,1,3);
% hold on;
% plot(time, curData.magnet.X/100, 'r');
% plot(time, curData.magnet.Y/100, 'g');
% plot(time, curData.magnet.Z/100, 'b'); 
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Flux (G)');
% title('Magnetometer');
% hold off;
% linkaxes(axis, 'x');
% 
% %% Process sensor data through algorithm
% f = 128;
% AHRS = MadgwickAHRS('SamplePeriod', 1/f, 'Beta', 0.01);
% % AHRS = MahonyAHRS('SamplePeriod', 1/f, 'Kp', 0.5);
% 
% quaternion = zeros(length(time), 4);
% Gyroscope = [curData.gyro.Z, curData.gyro.Y, curData.gyro.X];
% Accelerometer = [curData.accel.Z/9.8, curData.accel.Y/9.8, curData.accel.X/9.8];
% Magnetometer = [curData.magnet.Z/100, curData.magnet.Y/100, curData.magnet.X/100];
% 
% for t = 1:length(time)
%     AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
%     quaternion(t, :) = AHRS.Quaternion;
% end
% 
% %% Plot algorithm output as Euler angles
% % The first and third Euler angles in the sequence (phi and psi) become
% % unreliable when the middle angles of the sequence (theta) approaches ?0
% % degrees. This problem commonly referred to as Gimbal Lock.
% % See: http://en.wikipedia.org/wiki/Gimbal_lock
% 
% euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
% 
% figure('Name', 'Euler Angles');
% hold on;
% plot(time, euler(:,1), 'r');
% plot(time, euler(:,2), 'g');
% plot(time, euler(:,3), 'b');
% title('Euler angles');
% xlabel('Time (s)');
% ylabel('Angle (deg)');
% legend('\phi', '\theta', '\psi');
% hold off;

%% End of script