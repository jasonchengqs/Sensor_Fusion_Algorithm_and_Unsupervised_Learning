% This function was written by Qisen Cheng (qisench@umich.edu).
% Updated 2/1/17
% This function estimates orientation using complementary filtering.
% refernece: "the balance filter" by Shane Colton, 2007
% -------------------------------------------------------------------------
% inputs: imu->IMU dataset; comboId->id of dancer movements combo; 
%         highPass->highpass coeff of gyro data;
%         lowPass->lowpass coeff of accel data;
%         f->sampling frequency(128Hz)
%         session->command chosing "calibration" or "trials" sessions to be
%         processed
%         sigFrame->data reference frame, 'navi' or 'sensor'.
% -------------------------------------------------------------------------
function [imu] = complementaryFilter(imu, comboId, highPass, lowPass, f, session, sigFrame)
% comboId = 1;
% highPass = 0.98;
% lowPass = 0.02;
% f = 128; % Hz
dt = 1/f;
avg_yaw_bias = 4;
avg_window = 5;
names = fieldnames(imu);
if ~isempty(strfind(sigFrame, 'sensor'))
    for i = 1:length(names)
        for k = 1:length(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session))
            accX = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.accel.X;
            accX = movmean(accX, avg_window);
            accY = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.accel.Y;
            accX = movmean(accX, avg_window);
            accZ = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.accel.Z;
            accX = movmean(accX, avg_window);
            gyroX = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.gyro.X;
            gyroY = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.gyro.Y;
            gyroZ = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.gyro.Z;
            theta_X = zeros(size(accX));
            theta_Y = zeros(size(accX));
            theta_Z = zeros(size(accX));
            
            for t = 1:length(accX)
                if t == 1
%                     theta_X(t) = lowPass * atan2(accZ(t),accY(t));
                    theta_X(t) = 0;
                    theta_Y(t) = lowPass * atan2(accZ(t),accX(t));
                    theta_Z(t) = lowPass * atan2(accY(t),accX(t));                   
                end
                if t > 1
%                     theta_X(t) = highPass * (theta_X(t-1) + gyroX(t)*dt) + lowPass * (atan2(accZ(t),accY(t)) + 2*pi*(atan2(accZ(t),accY(t))<0));
%                     theta_Y(t) = highPass * (theta_Y(t-1) + gyroY(t)*dt) + lowPass * (atan2(accX(t),accZ(t)) + 2*pi*(atan2(accX(t),accZ(t))<0));
%                     theta_Z(t) = highPass * (theta_Z(t-1) + gyroZ(t)*dt) + lowPass * (atan2(accY(t),accX(t)) + 2*pi*(atan2(accY(t),accX(t))<0));                 
%                     theta_X(t) = highPass * (theta_X(t-1) + gyroX(t)*dt) + lowPass * (atan2(accZ(t),accY(t)));
                    theta_X(t) = theta_X(t-1) + (gyroX(t)-(avg_yaw_bias/f))*dt;
                    theta_Y(t) = highPass * (theta_Y(t-1) + gyroY(t)*dt) + lowPass * (atan2(accZ(t),accX(t)));
                    theta_Z(t) = highPass * (theta_Z(t-1) + gyroZ(t)*dt) + lowPass * (atan2(accY(t),accX(t)));
                end
            end
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.cmplt_angle.X = theta_X;
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.cmplt_angle.Y = theta_Y;
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.cmplt_angle.Z = theta_Z;
        end
    end
end

if ~isempty(strfind(sigFrame, 'navi'))
    for i = 1:length(names)
        for k = 1:length(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session))
            accX = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.accel.X;
            accX = movmean(accX, avg_window);
            accY = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.accel.Y;
            accY = movmean(accY, avg_window);
            accZ = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.accel.Z;
            accZ = movmean(accZ, avg_window);
            gyroX = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.gyro.X;
            gyroY = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.gyro.Y;
            gyroZ = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.gyro.Z;
            theta_X = zeros(size(accX));
            theta_Y = zeros(size(accX));
            theta_Z = zeros(size(accX));
            for t = 1:length(accX)
                if t == 1
%                     theta_X(t) = lowPass * atan2(accZ(t),accY(t));
                    theta_X(t) = 0;
                    theta_Y(t) = lowPass * atan2(accZ(t),accX(t));
                    theta_Z(t) = lowPass * atan2(accY(t),accX(t));
                end
                if t > 1
%                     theta_X(t) = highPass * (theta_X(t-1) + gyroX(t)*dt) + lowPass * atan2(accY(t),accZ(t));
                    theta_X(t) = theta_X(t-1) + (gyroX(t)-(avg_yaw_bias/f))*dt;
                    theta_Y(t) = highPass * (theta_Y(t-1) + gyroY(t)*dt) + lowPass * atan2(accZ(t),accX(t));
                    theta_Z(t) = highPass * (theta_Z(t-1) + gyroZ(t)*dt) + lowPass * atan2(accY(t),accX(t));
                end
            end
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.cmplt_angle.X = theta_X;
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.cmplt_angle.Y = theta_Y;
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.cmplt_angle.Z = theta_Z;
        end
    end
end