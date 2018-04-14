% This function was written by Qisen Cheng (qisench@umich.edu).
% Updated 2/15/17
% This function compensates (cancels) gravity from acceleration in 
% navigation frame, and saves the compensated acceleration under the cell 
% of navigation frame. Then it converts the acceleration back to 
% representations in sensor frame and saves under cell of sensor frame.

function [imu] = gravityCompensate(imu, comboId, session)
names = fieldnames(imu);
gravity = 9.8; % gravity is assumed to be 9.8 m/s^2

for k = 1:length(names)
    target = names{k};
    combo = strcat('combo_',num2str(comboId));
    curData = imu.(target).data.(combo);
    
    for i = 1:length(curData.(session))

        % read the acceleration in navigation frame
        NX = curData.(session)(i).navi_frame.accel.X;
        NX = NX - gravity; % gravity compensation
        NY = curData.(session)(i).navi_frame.accel.Y;
        NZ = curData.(session)(i).navi_frame.accel.Z;
        SX = zeros(size(curData.(session)(i).navi_frame.accel.X));
        SY = zeros(size(curData.(session)(i).navi_frame.accel.Y));
        SZ = zeros(size(curData.(session)(i).navi_frame.accel.Z));
        % save the compensated acceleration in navigation frame
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.noG_accel.X = NX;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.noG_accel.Y = NY;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).navi_frame.noG_accel.Z = NZ;        
        
        temp = NX;
        NZ = -NZ;
        NX = NZ;
        NZ = temp;
        for j = 1:length(curData.(session)(i).navi_frame.raw_quat.q1)
            
            % calculate the rotation matrix from quaternion in sensor frame
            q1 = curData.(session)(i).navi_frame.raw_quat.q1(j); q2 = curData.(session)(i).navi_frame.raw_quat.q2(j);
            q3 = curData.(session)(i).navi_frame.raw_quat.q3(j); q4 = curData.(session)(i).navi_frame.raw_quat.q4(j);
            C11 = q1^2 + q2^2 - q3^2 - q4^2; C12 = 2*(q2*q3 - q1*q4); C13 = 2*(q2*q4 + q1*q3);
            C21 = 2*(q2*q3 + q1*q4); C22 = q1^2 - q2^2 + q3^2 - q4^2; C23 = 2*(q3*q4 - q1*q2);
            C31 = 2*(q2*q4 - q1*q3); C32 = 2*(q3*q4 + q1*q2); C33 = q1^2 - q2^2 - q3^2 + q4^2;
            C = [C11, C12, C13;
                 C21, C22, C23;
                 C31, C32, C33];
            % calculate the reverse rotation matrix
            R = inv(C);
            R11 = R(1,1); R12 = R(1,2); R13 = R(1,3);
            R21 = R(2,1); R22 = R(2,2); R23 = R(2,3);
            R31 = R(3,1); R32 = R(3,2); R33 = R(3,3);
            
            % Convert the acceleration back to representations in sensor frame
            SX(j) = R11*NX(j) + R12*NY(j) + R13*NZ(j);
            SY(j) = R21*NX(j) + R22*NY(j) + R23*NZ(j);
            SZ(j) = R31*NX(j) + R32*NY(j) + R33*NZ(j);
        end
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).sensor_frame.noG_accel.X = SX;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).sensor_frame.noG_accel.Y = SY;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(session)(i).sensor_frame.noG_accel.Z = SZ;
    end
end