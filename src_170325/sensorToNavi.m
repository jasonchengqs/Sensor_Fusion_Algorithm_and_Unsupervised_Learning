% This function was written by Qisen Cheng (qisench@umich.edu).
% Updated 2/1/17
% This function converts acceleration and angular velocity from sensor frame to navigation frame

function [imu] = sensorToNavi(imu, comboId, session)
names = fieldnames(imu);
for k = 1:length(names)
    target = names{k};
    combo = strcat('combo_',num2str(comboId));
    curData = imu.(target).data.(combo);
    for i = 1:length(curData.(session))
        
        % initialize the acceleration/gyro in navigation frame. i.e NX
        % means acceleration along x in navigation frame, NRX means angular
        % rate about x in navigation frame.
        NX = zeros(size(curData.(session)(i).sensor_frame.accel.X));
        NY = zeros(size(curData.(session)(i).sensor_frame.accel.Y));
        NZ = zeros(size(curData.(session)(i).sensor_frame.accel.Z));
        NRX = zeros(size(curData.(session)(i).sensor_frame.gyro.X));
        NRY = zeros(size(curData.(session)(i).sensor_frame.gyro.Y));
        NRZ = zeros(size(curData.(session)(i).sensor_frame.gyro.Z));
        NMX = zeros(size(curData.(session)(i).sensor_frame.magnet.X));
        NMY = zeros(size(curData.(session)(i).sensor_frame.magnet.Y));
        NMZ = zeros(size(curData.(session)(i).sensor_frame.magnet.Z));
        for j = 1:length(curData.(session)(i).navi_frame.raw_quat.q1)
            
            % calculate the rotation matrix R (navi=R*sensor) from quaternion in sensor frame
            q1 = curData.(session)(i).navi_frame.raw_quat.q1(j);
            q2 = curData.(session)(i).navi_frame.raw_quat.q2(j);
            q3 = curData.(session)(i).navi_frame.raw_quat.q3(j);
            q4 = curData.(session)(i).navi_frame.raw_quat.q4(j);
            % CXX: entries in the rotation matrix R
            C11 = q1^2 + q2^2 - q3^2 - q4^2;
            C12 = 2*(q2*q3 - q1*q4);
            C13 = 2*(q2*q4 + q1*q3);
            C21 = 2*(q2*q3 + q1*q4);
            C22 = q1^2 - q2^2 + q3^2 - q4^2;
            C23 = 2*(q3*q4 - q1*q2);
            C31 = 2*(q2*q4 - q1*q3);
            C32 = 2*(q3*q4 + q1*q2);
            C33 = q1^2 - q2^2 - q3^2 + q4^2;
            
            % Convert the acceleration axis into navigation frame
            NX(j) = C11*curData.(session)(i).sensor_frame.accel.X(j)+...
                C12*curData.(session)(i).sensor_frame.accel.Y(j)+...
                C13*curData.(session)(i).sensor_frame.accel.Z(j);
            NY(j) = C21*curData.(session)(i).sensor_frame.accel.X(j)+...
                C22*curData.(session)(i).sensor_frame.accel.Y(j)+...
                C23*curData.(session)(i).sensor_frame.accel.Z(j);
            NZ(j) = C31*curData.(session)(i).sensor_frame.accel.X(j)+...
                C32*curData.(session)(i).sensor_frame.accel.Y(j)+...
                C33*curData.(session)(i).sensor_frame.accel.Z(j);
            
            % Convert the rotational axis into navi frame
            NRX(j) = C11*curData.(session)(i).sensor_frame.gyro.X(j)+...
                C12*curData.(session)(i).sensor_frame.gyro.Y(j)+...
                C13*curData.(session)(i).sensor_frame.gyro.Z(j);
            NRY(j) = C21*curData.(session)(i).sensor_frame.gyro.X(j)+...
                C22*curData.(session)(i).sensor_frame.gyro.Y(j)+...
                C23*curData.(session)(i).sensor_frame.gyro.Z(j);
            NRZ(j) = C31*curData.(session)(i).sensor_frame.gyro.X(j)+...
                C32*curData.(session)(i).sensor_frame.gyro.Y(j)+...
                C33*curData.(session)(i).sensor_frame.gyro.Z(j);
            
             % Convert the magnet axis into navi frame
            NMX(j) = C11*curData.(session)(i).sensor_frame.magnet.X(j)+...
                C12*curData.(session)(i).sensor_frame.magnet.Y(j)+...
                C13*curData.(session)(i).sensor_frame.magnet.Z(j);
            NMY(j) = C21*curData.(session)(i).sensor_frame.magnet.X(j)+...
                C22*curData.(session)(i).sensor_frame.magnet.Y(j)+...
                C23*curData.(session)(i).sensor_frame.magnet.Z(j);
            NMZ(j) = C31*curData.(session)(i).sensor_frame.magnet.X(j)+...
                C32*curData.(session)(i).sensor_frame.magnet.Y(j)+...
                C33*curData.(session)(i).sensor_frame.magnet.Z(j);
            
        end
        % save the data
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.accel.X = NZ;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.accel.Y = NY;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.accel.Z = -NX;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.gyro.X = NRZ;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.gyro.Y = NRY;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.gyro.Z = -NRX;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.magnet.X = NMZ;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.magnet.Y = NMY;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.magnet.Z = -NMX;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.time = ...
            imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).sensor_frame.time;
    end
end