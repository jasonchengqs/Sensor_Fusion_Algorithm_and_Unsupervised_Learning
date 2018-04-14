clear
clc
close all
cd(fileparts(mfilename('fullpath')))

%% data processing
imu = dataLoad(); % load the latest data file into the working space

% % transform the data from sensor frame to navigation frame
% imu = sensorToNavi(imu,1,'calibration');
% imu = sensorToNavi(imu,1,'trial');
% 
% % integrate the angular rate to calculate the orientation (roll/pitch/yaw) 
% imu = integAngularRate(imu, 1, 128, 'trial', ['sensor', 'navi']);
% imu = integAngularRate(imu, 1, 128, 'calibration', ['sensor', 'navi']);
% 
% % calculate the gravity-free acceleration in both sensor frame and
% % navigation frame
% imu = gravityCompensate(imu, 1, 'calibration');
% imu = gravityCompensate(imu, 1, 'trial');
% 
% % orientation estimation using complementary filtering
% imu = complementaryFilter(imu, 1, 0.99, 0.01, 128, 'trial',['sensor', 'navi']);
% imu = complementaryFilter(imu, 1, 0.99, 0.01, 128, 'calibration',['sensor', 'navi']);
% 
% % orientation estimation using kalman filtering
% imu = kalmanFilter(imu, 1, 128, 'trial', ['sensor', 'navi']);
% imu = kalmanFilter(imu, 1, 128, 'calibration', ['sensor', 'navi']);
% 
% % orientation estimation using madgwick algorithm
% imu = madgwick(imu, 1, 128, 0.1, 'trial');
% imu = madgwick(imu, 1, 128, 0.1, 'calibration');

saveData(imu); % save the processed data file -- 'imu_dd_mm_yyyy.mat'

%% data ploting
% dataPlot(imu,'combo_1', 'Jill', 'trial', ['integ_angles_navi','cmplt_angles_navi','kalman_angles_navi','madgwick_angles_navi'])
% dataPlot(imu,'combo_1', 'Jill', 'trial', ['kalman_angles_navi'])
% dataPlot(imu,'combo_1', 'Elius', 'trial', ['integ_angles_navi','cmplt_angles_navi','kalman_angles_navi','madgwick_angles_navi'])
dataPlot(imu,'combo_1', 'Rob', 'trial', ['kalman_angles_sensor'])
dataPlot(imu,'combo_1', 'Jill', 'trial', ['kalman_angles_sensor'])
randId_DTW = randi([1 6], 1, 2); 
config_DTW = [{'Jill', 'Luby'}, 1, 'trial', randId_DTW, 'sensor_frame', 'kalman_angle', 'Z'];
sigDTW(imu, 128, config_DTW);
