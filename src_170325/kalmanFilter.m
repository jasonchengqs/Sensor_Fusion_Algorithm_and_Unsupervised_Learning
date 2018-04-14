% This function was written by Qisen Cheng (qisench@umich.edu).
% Updated 2/1/17
% This function estimates orientation using complementary filtering.
% refernece: "A practical approach to Kalman filter and how to implement
% it" by Kristian, 2013
% -------------------------------------------------------------------------
% inputs: imu->IMU dataset; comboId->id of dancer movements combo; 
%         f->sampling frequency(128Hz)
%         session->command chosing "calibration" or "trials" sessions to be
%         processed
%         sigFrame->data reference frame, 'navi' or 'sensor'.
% -------------------------------------------------------------------------

function [imu] = kalmanFilter(imu, comboId, f, session, sigFrame)
% sigFrame = {'navi'};
% session = 'calibration';
% f = 128;
dt = 1/f;
names = fieldnames(imu);

Q_angle = 1e-5; % reflects strong trust in the angle estimation
Q_gyroBias = 3e-5; % reflects strong trust in the calibrated (by IMU itself) gyro rates
R = 0.03; % reflects relatively weak trust in the observation of accelerations
initL = 0; % assign the initial condition (initial angles) to be zero with high confidence

avg_window = 5;
avg_yaw_bias = 4;

if ~isempty(strfind(sigFrame, 'navi'))
    for i = 1:length(names)
        curData = imu.(names{i}).data.(strcat('combo_',num2str(comboId)));
        for k = 1:length(curData.(session))
            % pre-1: initializing variables
            accX = curData.(session)(k).navi_frame.accel.X;
            accX = movmean(accX, avg_window);
            accY = curData.(session)(k).navi_frame.accel.Y;
            accY = movmean(accY, avg_window);
            accZ = curData.(session)(k).navi_frame.accel.Z;
            accZ = movmean(accZ, avg_window);
            gyroX = curData.(session)(k).navi_frame.gyro.X;
            gyroY = curData.(session)(k).navi_frame.gyro.Y;
            gyroZ = curData.(session)(k).navi_frame.gyro.Z;
            theta_accX = zeros(size(accX));
            theta_accY = zeros(size(accY));
            theta_accZ = zeros(size(accZ));
            
            % pre-2: calculate the angles (pitch/roll) from acceleration data
            for t = 1:length(accX)
                if t == 1
                    theta_accX(t) = 0;
                    theta_accY(t) = 0;
                    theta_accZ(t) = 0;
                end
                if t > 1
%                   theta_accX(t) = atan2(accZ(t),accY(t));
                    theta_accX(t) = theta_accX(t-1) + (gyroX(t)-(avg_yaw_bias/f))*dt;
                    theta_accY(t) = atan2(accZ(t),accX(t));
                    theta_accZ(t) = atan2(accY(t),accX(t));
                end
            end
            
            % kalman-filtering
            angRateBias_0 = 0;
            angle_0 = 0;
            for iAngle = 1:3
                if iAngle == 1
                    gyro = gyroX;
                    theta_acc = theta_accX;
                    acc = accX;
                    else if iAngle == 2
                        gyro = gyroY;
                        theta_acc = theta_accY;
                        acc = accY;
                    else if iAngle == 3
                        gyro = gyroZ;
                        theta_acc = theta_accZ;
                        acc = accZ;
                        end   
                    end
                end
                bias = zeros(size(gyroX)); angRate = zeros(size(gyroX)); angle = zeros(size(gyroX));
                P11 = zeros(size(gyroX)); P12 = zeros(size(gyroX)); P21 = zeros(size(gyroX)); P22 = zeros(size(gyroX));
                err = zeros(size(gyroX)); S = zeros(size(gyroX)); K1 = zeros(size(gyroX)); K2 = zeros(size(gyroX));
                for t = 1:length(accX)
                    if t == 1
                        bias(t) = angRateBias_0;
                        angRate(t) = gyro(t)-bias(t);
                        angle(t) = angle_0 + angRate(t)*dt;
                        P11(t) = initL; P12(t) = 0; P21(t) = 0; P22(t) = initL;
                        err(t) = theta_acc(t) - angle(t);
                        S(t) = P11(t) + R;
                        K1(t) = P11(t) / S(t);
                        K2(t) = P21(t) / S(t);
                    end
                    if t > 1
                        bias(t) = bias(t-1);
                        angRate(t) = gyro(t) - bias(t);
                        angle(t) = angle(t-1) + angRate(t)*dt;
                        P22(t) = P22(t-1) + Q_gyroBias*dt;
                        P21(t) = P21(t-1) - P22(t)*dt;
                        P12(t) = P12(t-1) - P22(t)*dt;
                        P11(t) = P11(t-1) + (P22(t)*dt - P12(t) - P21(t) + Q_angle)*dt;
                        % update from the prediction (prior)
                        err(t) = theta_acc(t) - angle(t);
                        S(t) = P11(t) + R;
                        K1(t) = P11(t) / S(t); % kalman gain
                        K2(t) = P21(t) / S(t); 
                        % linear combination the prior and err
                        angle(t) = angle(t) + K1(t)*err(t);
                        bias(t) = bias(t) + K2(t)*err(t);
                        % update covariance matrix
                        P11_temp = P11(t);  P12_temp = P12(t);
                        P11(t) = P11(t) - K1(t)*P11_temp; 
                        P12(t) = P12(t) - K1(t)*P12_temp;
                        P21(t) = P21(t) - K2(t)*P11_temp;
                        P22(t) = P22(t) - K2(t)*P12_temp;
                    end
               end
               if iAngle == 1
                    imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.kalman_angle.X = angle;
                    imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.kalman_gyro_bias.X = bias;
                    else if iAngle == 2
                        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.kalman_angle.Y = angle;
                        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.kalman_gyro_bias.Y = bias;
                    else if iAngle == 3
                        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.kalman_angle.Z = angle;
                        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).navi_frame.kalman_gyro_bias.Z = bias;
                        end   
                    end
                end 
            end
        end
    end
end

if ~isempty(strfind(sigFrame, 'sensor'))
    for i = 1:length(names)
        curData = imu.(names{i}).data.(strcat('combo_',num2str(comboId)));
        for k = 1:length(curData.(session))
            % pre-1: initializing variables
            accX = curData.(session)(k).navi_frame.accel.X;
            accX = movmean(accX, avg_window);
            accY = curData.(session)(k).navi_frame.accel.Y;
            accY = movmean(accY, avg_window);
            accZ = curData.(session)(k).navi_frame.accel.Z;
            accZ = movmean(accZ, avg_window);
            gyroX = curData.(session)(k).sensor_frame.gyro.X;
            gyroY = curData.(session)(k).sensor_frame.gyro.Y;
            gyroZ = curData.(session)(k).sensor_frame.gyro.Z;
            theta_accX = zeros(size(accX));
            theta_accY = zeros(size(accY));
            theta_accZ = zeros(size(accZ));
            
            % pre-2: calculate the angles (pitch/roll) from acceleration data
            for t = 1:length(accX)
                if t == 1
                    theta_accX(t) = 0;
                    theta_accY(t) = 0;
                    theta_accZ(t) = 0;
                end
                if t > 1
%                   theta_accX(t) = atan2(accZ(t),accY(t));
                    theta_accX(t) = theta_accX(t-1) + (gyroX(t)-(avg_yaw_bias/f))*dt;
                    theta_accY(t) = atan2(accZ(t),accX(t));
                    theta_accZ(t) = atan2(accY(t),accX(t));
                end
            end
            
            % kalman-filtering
            angRateBias_0 = 0;
            angle_0 = 0;
            for iAngle = 1:3
                if iAngle == 1
                    gyro = gyroX;
                    theta_acc = theta_accX;
                    acc = accX;
                    else if iAngle == 2
                        gyro = gyroY;
                        theta_acc = theta_accY;
                        acc = accY;
                    else if iAngle == 3
                        gyro = gyroZ;
                        theta_acc = theta_accZ;
                        acc = accZ;
                        end   
                    end
                end
                bias = zeros(size(gyroX)); angRate = zeros(size(gyroX)); angle = zeros(size(gyroX));
                P11 = zeros(size(gyroX)); P12 = zeros(size(gyroX)); P21 = zeros(size(gyroX)); P22 = zeros(size(gyroX));
                err = zeros(size(gyroX)); S = zeros(size(gyroX)); K1 = zeros(size(gyroX)); K2 = zeros(size(gyroX));
                for t = 1:length(accX)
                    if t == 1
                        bias(t) = angRateBias_0;
                        angRate(t) = gyro(t)-bias(t);
                        angle(t) = angle_0 + angRate(t)*dt;
                        P11(t) = initL; P12(t) = 0; P21(t) = 0; P22(t) = initL;
                        err(t) = theta_acc(t) - angle(t);
                        S(t) = P11(t) + R;
                        K1(t) = P11(t) / S(t);
                        K2(t) = P21(t) / S(t);
                    end
                    if t > 1
                        bias(t) = bias(t-1);
                        angRate(t) = gyro(t) - bias(t);
                        angle(t) = angle(t-1) + angRate(t)*dt;
                        P22(t) = P22(t-1) + Q_gyroBias*dt;
                        P21(t) = P21(t-1) - P22(t)*dt;
                        P12(t) = P12(t-1) - P22(t)*dt;
                        P11(t) = P11(t-1) + (P22(t)*dt - P12(t) - P21(t) + Q_angle)*dt;
                        % update from the prediction (prior)
                        err(t) = theta_acc(t) - angle(t);
                        S(t) = P11(t) + R;
                        K1(t) = P11(t) / S(t); % kalman gain
                        K2(t) = P21(t) / S(t); 
                        % linear combination the prior and err
                        angle(t) = angle(t) + K1(t)*err(t);
                        bias(t) = bias(t) + K2(t)*err(t);
                        % update covariance matrix
                        P11_temp = P11(t);  P12_temp = P12(t);
                        P11(t) = P11(t) - K1(t)*P11_temp; 
                        P12(t) = P12(t) - K1(t)*P12_temp;
                        P21(t) = P21(t) - K2(t)*P11_temp;
                        P22(t) = P22(t) - K2(t)*P12_temp;
                    end
               end
               if iAngle == 1
                    imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.kalman_angle.X = angle;
                    imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.kalman_gyro_bias.X = bias;
                    else if iAngle == 2
                        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.kalman_angle.Y = angle;
                        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.kalman_gyro_bias.Y = bias;
                    else if iAngle == 3
                        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.kalman_angle.Z = angle;
                        imu.(names{i}).data.(strcat('combo_',num2str(comboId))).(session)(k).sensor_frame.kalman_gyro_bias.Z = bias;
                        end   
                    end
                end 
            end
        end
    end
end
% discrete = -1;
% sys = ss(A,B,C,0,discrete,'inputname',{'u' 'w' 'v'},'outputname',{'y' 'yv'); 
% [kalmf, L,~, M, Z] = kalman(sys, Q, R);
