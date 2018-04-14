% This function was written by Qisen Cheng (qisench@umich.edu).
% Updated 1/10/17
% This function initializes a data structure containing raw
% acceleration, angular velocity, quaternions, magenetometer data in sensor frame,

function strucInit
clear
clc
close all

%% Define the data files for processing
rawData = {'20160511-181416_Jill_sensor_data_monitor_2772_label_sacrum.mat',...
           '20160511-184911_Luby_sensor_data_monitor_2772_label_sacrum.mat',...
           '20160511-192023_Rob_sensor_data_monitor_2772_label_sacrum.mat',...
           '20160601-183402_Elius_sensor_data_monitor_2772_label_sacrum.mat',...
           '20160601-185613_Wei_sensor_data_monitor_2772_label_sacrum.mat'};

%% Define the time window of trials
timeWinCalib = {[8.924e4,9.128e4; 1.256e5,1.271e5]; ...
                [7.151e4,7.299e4]; ...
                [2.714e4,2.854e4]; ...
                [5.533e4,5.647e4; 5.73e4,5.796e4]; ...
                [2.834e4,3.082e4; 3.54e4,3.698e4]
                };
timeWinTrials = {[1.297e5,1.327e5; 1.436e5,1.465e5; 1.465e5,1.493e5; ...
                  1.493e5,1.521e5; 1.521e5,1.549e5; 1.549e5,1.577e5];
                 [7.416e4,7.758e4; 7.758e4,8.139e4; 8.139e4,8.518e4; ...
                  8.518e4,8.898e4; 8.898e4,9.272e4; 9.272e4,9.646e4; ];
                 [2.162e4,2.496e4; 3.241e4,3.580e4; 3.580e4,3.959e4; ...
                  3.959e4,4.331e4; 4.331e4,4.713e4; 4.713e4,5.087e4; 5.087e4,5.467e4];
                 [6.714e4,7.090e4; 7.090e4,7.469e4; 7.469e4,7.845e4; ...
                  7.845e4,8.222e4; 8.222e4,8.605e4; 8.605e4,8.986e4];
                 [4.223e4,4.601e4; 4.601e4,5.074e4; 5.074e4,5.547e4; ...
                  5.547e4,6.024e4; 6.024e4,6.494e4; 6.494e4,6.97e4]
                  };

%% Extract "rawDataSourceName", "rawDataDate", and "rawDataTime", and "dancerName".
for i = 1:length(rawData)
    rawDataSourceName(i).name = rawData{i};
    file = rawDataSourceName(i).name;
    for j = 1:length(file)
        rawDataDate(i).date = file(1:8);
        rawDataTime(i).time = file(10:15);
        chrPointer = 17;
        dancerName(i).name = '';
        while (file(chrPointer) ~= '_')
            dancerName(i).name = strcat(dancerName(i).name, file(chrPointer));
            chrPointer = chrPointer + 1;
        end
    end
end

%% Organize all the data into a structure named "IMU" and save as .mat file.
for i = 1:length(rawData)
    load(rawData{i})
    imu.(dancerName(i).name).rawDataSourceName = rawDataSourceName(i).name;
    imu.(dancerName(i).name).date = rawDataDate(i).date;
    imu.(dancerName(i).name).startTime = rawDataTime(i).time;
    
    % calibration sessions
    for k = 1:size(timeWinCalib{i},1)
        calibStart = timeWinCalib{i}(k,1);
        calibStop = timeWinCalib{i}(k,2);
        % acceleration
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).sensor_frame.accel.X...
            = IMU.sacrum.a(calibStart:calibStop,1);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).sensor_frame.accel.Y...
            = IMU.sacrum.a(calibStart:calibStop,2);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).sensor_frame.accel.Z...
            = IMU.sacrum.a(calibStart:calibStop,3);
        % gyroscope
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).sensor_frame.gyro.X...
            = IMU.sacrum.w(calibStart:calibStop,1);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).sensor_frame.gyro.Y...
            = IMU.sacrum.w(calibStart:calibStop,2);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).sensor_frame.gyro.Z...
            = IMU.sacrum.w(calibStart:calibStop,3);
        % magenetometer
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).sensor_frame.magnet.X...
            = IMU.sacrum.m(calibStart:calibStop,1);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).sensor_frame.magnet.Y...
            = IMU.sacrum.m(calibStart:calibStop,2);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).sensor_frame.magnet.Z...
            = IMU.sacrum.m(calibStart:calibStop,3);
        % quaternion
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).navi_frame.raw_quat.q1...
            = IMU.sacrum.q(calibStart:calibStop,1);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).navi_frame.raw_quat.q2...
            = IMU.sacrum.q(calibStart:calibStop,2);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).navi_frame.raw_quat.q3...
            = IMU.sacrum.q(calibStart:calibStop,3);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).navi_frame.raw_quat.q4...
            = IMU.sacrum.q(calibStart:calibStop,4);
        quaternion = IMU.sacrum.q(calibStart:calibStop,:);
        euler = quatern2euler(quaternConj(quaternion));
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).navi_frame.raw_angle.X = ...
            euler(:,3);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).navi_frame.raw_angle.Y = ...
            euler(:,2);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).navi_frame.raw_angle.Z = ...
            euler(:,1);
        % time
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).calibration(k).sensor_frame.time...
            = IMU.time(calibStart:calibStop,1);
    end
    
    % trials
    for k = 1:size(timeWinTrials{i},1)
        start = timeWinTrials{i}(k,1);
        stop = timeWinTrials{i}(k,2);
        % acceleration
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).sensor_frame.accel.X...
            = IMU.sacrum.a(start:stop,1);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).sensor_frame.accel.Y...
            = IMU.sacrum.a(start:stop,2);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).sensor_frame.accel.Z...
            = IMU.sacrum.a(start:stop,3);
        % gyroscope
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).sensor_frame.gyro.X...
            = IMU.sacrum.w(start:stop,1);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).sensor_frame.gyro.Y...
            = IMU.sacrum.w(start:stop,2);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).sensor_frame.gyro.Z...
            = IMU.sacrum.w(start:stop,3);
        % magenetometer
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).sensor_frame.magnet.X...
            = IMU.sacrum.m(start:stop,1);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).sensor_frame.magnet.Y...
            = IMU.sacrum.m(start:stop,2);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).sensor_frame.magnet.Z...
            = IMU.sacrum.m(start:stop,3);
        % quaternion
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).navi_frame.raw_quat.q1...
            = IMU.sacrum.q(start:stop,1);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).navi_frame.raw_quat.q2...
            = IMU.sacrum.q(start:stop,2);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).navi_frame.raw_quat.q3...
            = IMU.sacrum.q(start:stop,3);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).navi_frame.raw_quat.q4...
            = IMU.sacrum.q(start:stop,4);
        quaternion = IMU.sacrum.q(start:stop,:);
        euler = quatern2euler(quaternConj(quaternion));
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).navi_frame.raw_angle.X = ...
            euler(:,3);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).navi_frame.raw_angle.Y = ...
            euler(:,2);
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).navi_frame.raw_angle.Z = ...
            euler(:,1);
        % time
        imu.(dancerName(i).name).data.(strcat('combo_',num2str(1))).trial(k).sensor_frame.time...
            = IMU.time(start:stop,1);
    end
end

%% Save data
updateMark = datestr(datetime('today'));
fileName = strcat('imu_',updateMark,'.mat');
save(fileName, 'imu');

% %% Set the default colors for plotting
% co = [0         0    1.0000;
%      1.0000         0         0;
%     0.9290    0.6940    0.1250;
%          0    0.7500    0.7500;
%     0.7500         0    0.7500;
%     0.7500    0.7500         0;
%     0.2500    0.2500    0.2500];
% set(groot,'defaultAxesColorOrder',co)


% %% Jill data plot
% target = 'Elius';
% combo = 'combo_1';
% figure('Name',strcat(target,'_data'))
% 
% curData = imu.(target).data.(combo);
% for p = 1:2
%     figure (p);
%     for j = 1:length(curData.trial)
%         subplot(3,ceil(length(curData.trial)/3),j);
%         hold on
%         if (p == 1)
%             plot(curData.trial(j).acceleration.X);
%             plot(curData.trial(j).acceleration.Y);
%             plot(curData.trial(j).acceleration.Z);
%             legend('acc-X','acc-Y','acc-Z')
%         else
%             plot(curData.trial(j).gyro.X);
%             plot(curData.trial(j).gyro.Y);
%             plot(curData.trial(j).gyro.Z);
%             legend('gyro-X','gyro-Y','gyro-Z')
%         end
%     end
% end