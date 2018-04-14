function [imu] = integAngularRate(imu, comboId, f, session, sigFrame)
% comboId = 1;
% f = 128; % Hz
dt = 1/f;
names = fieldnames(imu);
if ~isempty(strfind(sigFrame, 'sensor'))
    for i = 1:length(names)
        for k = 1:length(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session))
            accX = detrend(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.accel.X);
            accY = detrend(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.accel.Y);
            accZ = detrend(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.accel.Z);
            gyroX = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.gyro.X;
            gyroY = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.gyro.Y;
            gyroZ = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.gyro.Z;
            theta_X = zeros(size(accX));
            theta_Y = zeros(size(accX));
            theta_Z = zeros(size(accX));
            for t = 1:length(accX)
                if t == 1
                    theta_X(t) = 0;
                    theta_Y(t) = 0;
                    theta_Z(t) = 0;
                end
                if t > 1
                    theta_X(t) = theta_X(t-1) + gyroX(t)*dt;
                    theta_Y(t) = theta_Y(t-1) + gyroY(t)*dt;
                    theta_Z(t) = theta_Z(t-1) + gyroZ(t)*dt;
                end
            end
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.inteAngles.X = theta_X;
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.inteAngles.Y = theta_Y;
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.inteAngles.Z = theta_Z;
        end
    end
end

if ~isempty(strfind(sigFrame, 'navi'))
    for i = 1:length(names)
        for k = 1:length(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session))
            accX = detrend(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.accel.X);
            accY = detrend(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.accel.Y);
            accZ = detrend(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.accel.Z);
            gyroX = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.gyro.X;
            gyroY = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.gyro.Y;
            gyroZ = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.gyro.Z;
            theta_X = zeros(size(accX));
            theta_Y = zeros(size(accX));
            theta_Z = zeros(size(accX));
            for t = 1:length(accX)
                if t == 1
                    theta_X(t) = 0;
                    theta_Y(t) = 0;
                    theta_Z(t) = 0;
                end
                if t > 1
                    theta_X(t) = theta_X(t-1) + gyroX(t)*dt;
                    theta_Y(t) = theta_Y(t-1) + gyroY(t)*dt;
                    theta_Z(t) = theta_Z(t-1) + gyroZ(t)*dt;
                end
            end
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.inteAngles.X = theta_X;
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.inteAngles.Y = theta_Y;
            imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.inteAngles.Z = theta_Z;
        end
    end
end

%%
% if ~isempty(strfind(cmd, 'raw'))
%     for i = 1:length(names)
%         for k = 1:length(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session))
%             accX = detrend(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).acceleration.X);
%             accY = detrend(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).acceleration.Y);
%             accZ = detrend(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).acceleration.Z);
%             gyroX = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).gyro.X;
%             gyroY = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).gyro.Y;
%             gyroZ = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).gyro.Z;
%             theta_X = zeros(size(accX));
%             theta_Y = zeros(size(accX));
%             theta_Z = zeros(size(accX));
%             for t = 1:length(accX)
%                 if t == 1
%                     theta_X(t) = 0;
%                     theta_Y(t) = 0;
%                     theta_Z(t) = 0;
%                 end
%                 if t > 1
%                     if ~isempty(strfind(session, 'trial'))
%                         theta_X(t) = theta_X(t-1) + (gyroX(t)-0.03)*dt;
%                         theta_Y(t) = theta_Y(t-1) + (gyroY(t)+0.02)*dt;
%                         theta_Z(t) = theta_Z(t-1) + (gyroZ(t)-0.03)*dt;
%                     end
%                     if ~isempty(strfind(session, 'calibration'))
%                         theta_X(t) = theta_X(t-1) + gyroX(t)*dt;
%                         theta_Y(t) = theta_Y(t-1) + gyroY(t)*dt;
%                         theta_Z(t) = theta_Z(t-1) + gyroZ(t)*dt;
%                     end
%                 end
%             end
%             imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(opt)(k).angles.thetaX = detrend(theta_X,'linear');
%             imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(opt)(k).angles.thetaY = detrend(theta_Y,'linear');
%             imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(opt)(k).angles.thetaZ = detrend(theta_Z,'linear');
%             if ~isempty(strfind(session, 'trial'))
%                 imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).angles.thetaX = theta_X;
%                 imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).angles.thetaY = theta_Z;
%                 imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).angles.thetaZ = theta_Y;
%             end
%             if ~isempty(strfind(session, 'calibration'))
%                 imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).angles.thetaX = theta_X;
%                 imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).angles.thetaY = theta_Y;
%                 imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).angles.thetaZ = theta_Z;
%             end
%         end
%     end
% end
% 
% if ~isempty(strfind(cmd, 'navi'))
%     for i = 1:length(names)
%         for k = 1:length(imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session))
%             accX = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navigation.accNavi.X;
%             accY = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navigation.accNavi.Y;
%             accZ = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navigation.accNavi.Z;
%             gyroX = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navigation.gyroNavi.X;
%             gyroY = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navigation.gyroNavi.Y;
%             gyroZ = imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navigation.gyroNavi.Z;
%             theta_X = zeros(size(accX));
%             theta_Y = zeros(size(accX));
%             theta_Z = zeros(size(accX));
%             for t = 1:length(accX)
%                 if t == 1
%                     theta_X(t) = 0;
%                     theta_Y(t) = 0;
%                     theta_Z(t) = 0;
%                 end
%                 if t > 1
%                     theta_X(t) = theta_X(t-1) + (gyroX(t)-0.02)*dt;
%                     theta_Y(t) = theta_Y(t-1) + (gyroY(t)+0.02)*dt;
%                     theta_Z(t) = theta_Z(t-1) + (gyroZ(t)-0.02)*dt;
%                 end
%             end
%             imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navigation.angles.thetaX = detrend(theta_X,'linear');
%             imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navigation.angles.thetaY = detrend(theta_Y,'linear');
%             imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navigation.angles.thetaZ = detrend(theta_Z,'linear');
%         end
%     end
% end