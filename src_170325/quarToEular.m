function [imu] = quarToEular(imu,comboId)
names = fieldnames(imu);
for k = 1:length(names)
    target = names{k};
    combo = strcat('combo_',num2str(comboId));
    curData = imu.(target).data.(combo);
    for i = 1:length(curData.trial)
        theta_X = zeros(size(curData.trial(i).sensor_frame.quat.q1));
        theta_Y = zeros(size(curData.trial(i).sensor_frame.quat.q1));
        theta_Z = zeros(size(curData.trial(i).sensor_frame.quat.q1));
        thetaB_X = zeros(size(curData.trial(i).sensor_frame.quat.q1));
        thetaB_Y = zeros(size(curData.trial(i).sensor_frame.quat.q1));
        thetaB_Z = zeros(size(curData.trial(i).sensor_frame.quat.q1));
        for j = 1:length(curData.trial(i).quarternion.q1)
            q1 = curData.trial(i).quarternion.q1(j); q2 = curData.trial(i).quarternion.q2(j);
            q3 = curData.trial(i).quarternion.q3(j); q4 = curData.trial(i).quarternion.q4(j);
            C11 = q1^2 + q2^2 - q3^2 - q4^2; C12 = 2*(q2*q3 - q1*q4); C13 = 2*(q2*q4 + q1*q3);
            C21 = 2*(q2*q3 + q1*q4); C22 = q1^2 - q2^2 + q3^2 - q4^2; C23 = 2*(q3*q4 - q1*q2);
            C31 = 2*(q2*q4 - q1*q3); C32 = 2*(q3*q4 + q1*q2); C33 = q1^2 - q2^2 - q3^2 + q4^2;
            C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];
            I = inv(C);
%             I11 = I(1,1); I12 = I(1,2); I13 = I(1,3);
%             I21 = I(2,1); I22 = I(2,2); I23 = I(2,3);
%             I31 = I(3,1); I32 = I(3,2); I33 = I(3,3);
            iquat = zeros(1,4); qB = zeros(1,4); 
            iquat(1) = 0.5*sqrt(1+I(1,1)+I(2,2)+I(3,3)); iquat(2) = (I(3,2)-I(2,3))/(4*iquat(1));
            iquat(3) = (I(1,3)-I(3,1))/(4*iquat(1)); iquat(4) = (I(2,1)-I(1,2))/(4*iquat(1));
            iquatInv = iquat; iquatInv(2) = - iquatInv(2); iquatInv(3) = - iquatInv(3); iquatInv(4) = - iquatInv(4);
            qB(1) = q1*iquat(1) - q2*iquat(2) - q3*iquat(3) - q4*iquat(4);
            qB(2) = q1*iquat(2) + q2*iquat(1) - q3*iquat(4) + q4*iquat(3);
            qB(3) = q1*iquat(3) + q2*iquat(4) + q3*iquat(1) - q4*iquat(2);
            qB(4) = q1*iquat(4) - q2*iquat(3) + q3*iquat(2) + q4*iquat(1);
            
            qB(1) = iquatInv(1)*qB(1) - iquatInv(2)*qB(2) - iquatInv(3)*qB(3) - iquatInv(4)*qB(4);
            qB(2) = iquatInv(1)*qB(2) + iquatInv(2)*qB(1) - iquatInv(3)*qB(4) + iquatInv(4)*qB(3);
            qB(3) = iquatInv(1)*qB(3) + iquatInv(2)*qB(4) + iquatInv(3)*qB(1) - iquatInv(4)*qB(2);
            qB(4) = iquatInv(1)*qB(4) - iquatInv(2)*qB(3) + iquatInv(3)*qB(2) + iquatInv(4)*qB(1);
            
            qB1 = qB(1); qB2 = qB(2); qB3 = qB(3); qB4 = qB(4);
            CB11 = qB1^2 + qB2^2 - qB3^2 - qB4^2; CB12 = 2*(qB2*qB3 - qB1*qB4); CB13 = 2*(qB2*qB4 + qB1*qB3);
            CB21 = 2*(qB2*qB3 + qB1*qB4); CB22 = qB1^2 - qB2^2 + qB3^2 - qB4^2; CB23 = 2*(qB3*qB4 - qB1*qB2);
            CB31 = 2*(qB2*qB4 - qB1*qB3); CB32 = 2*(qB3*qB4 + qB1*qB2); CB33 = qB1^2 - qB2^2 - qB3^2 + qB4^2;
            theta_X(j) = atan2(C32, C33);
            if theta_X(j) < 0
                theta_X(j) = theta_X(j) + 2*pi;
            end
            theta_Y(j) = asin(-1*C31);
            if theta_Y(j) < 0
                theta_Y(j) = theta_Y(j) + 2*pi;
            end
            theta_Z(j) = atan2(C21, C11);
            if theta_Z(j) < 0
                theta_Z(j) = theta_Z(j) + 2*pi;
            end
            
            thetaB_X(j) = atan2(CB32, CB33);
%             if thetaB_X(j) < 0
%                 thetaB_X(j) = thetaB_X(j) + 2*pi;
%             end
            thetaB_Y(j) = asin(-1*CB31);
%             if thetaB_Y(j) < 0
%                 thetaB_Y(j) = thetaB_Y(j) + 2*pi;
%             end
            thetaB_Z(j) = atan2(CB21, CB11);
%             if thetaB_Z(j) < 0
%                 thetaB_Z(j) = thetaB_Z(j) + 2*pi;
%             end
        end
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).trial(i).euler.thetaX = theta_X;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).trial(i).euler.thetaY = theta_Y;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).trial(i).euler.thetaZ = theta_Z;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).trial(i).eulerB.thetaX = thetaB_Z;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).trial(i).eulerB.thetaY = -thetaB_Y;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).trial(i).eulerB.thetaZ = -thetaB_X;
    end
    
    for i = 1:length(curData.calibration)
        theta_X = zeros(size(curData.calibration(i).quarternion.q1));
        theta_Y = zeros(size(curData.calibration(i).quarternion.q1));
        theta_Z = zeros(size(curData.calibration(i).quarternion.q1));
        thetaB_X = zeros(size(curData.calibration(i).quarternion.q1));
        thetaB_Y = zeros(size(curData.calibration(i).quarternion.q1));
        thetaB_Z = zeros(size(curData.calibration(i).quarternion.q1));
        for j = 1:length(curData.calibration(i).quarternion.q1)
            q1 = curData.calibration(i).quarternion.q1(j); q2 = curData.calibration(i).quarternion.q2(j);
            q3 = curData.calibration(i).quarternion.q3(j); q4 = curData.calibration(i).quarternion.q4(j);
            C11 = q1^2 + q2^2 - q3^2 - q4^2; C12 = 2*(q2*q3 - q1*q4); C13 = 2*(q2*q4 + q1*q3);
            C21 = 2*(q2*q3 + q1*q4); C22 = q1^2 - q2^2 + q3^2 - q4^2; C23 = 2*(q3*q4 - q1*q2);
            C31 = 2*(q2*q4 - q1*q3); C32 = 2*(q3*q4 + q1*q2); C33 = q1^2 - q2^2 - q3^2 + q4^2;
            C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];
            I = inv(C);
%             I11 = I(1,1); I12 = I(1,2); I13 = I(1,3);
%             I21 = I(2,1); I22 = I(2,2); I23 = I(2,3);
%             I31 = I(3,1); I32 = I(3,2); I33 = I(3,3);
            iquat = zeros(1,4); qB = zeros(1,4); 
            iquat(1) = 0.5*sqrt(1+I(1,1)+I(2,2)+I(3,3)); iquat(2) = (I(3,2)-I(2,3))/(4*iquat(1));
            iquat(3) = (I(1,3)-I(3,1))/(4*iquat(1)); iquat(4) = (I(2,1)-I(1,2))/(4*iquat(1));
            iquatInv = iquat; iquatInv(2) = - iquatInv(2); iquatInv(3) = - iquatInv(3); iquatInv(4) = - iquatInv(4);
            qB(1) = q1*iquat(1) - q2*iquat(2) - q3*iquat(3) - q4*iquat(4);
            qB(2) = q1*iquat(2) + q2*iquat(1) - q3*iquat(4) + q4*iquat(3);
            qB(3) = q1*iquat(3) + q2*iquat(4) + q3*iquat(1) - q4*iquat(2);
            qB(4) = q1*iquat(4) - q2*iquat(3) + q3*iquat(2) + q4*iquat(1);
            
            qB(1) = iquatInv(1)*qB(1) - iquatInv(2)*qB(2) - iquatInv(3)*qB(3) - iquatInv(4)*qB(4);
            qB(2) = iquatInv(1)*qB(2) + iquatInv(2)*qB(1) - iquatInv(3)*qB(4) + iquatInv(4)*qB(3);
            qB(3) = iquatInv(1)*qB(3) + iquatInv(2)*qB(4) + iquatInv(3)*qB(1) - iquatInv(4)*qB(2);
            qB(4) = iquatInv(1)*qB(4) - iquatInv(2)*qB(3) + iquatInv(3)*qB(2) + iquatInv(4)*qB(1);
            
            qB1 = qB(1); qB2 = qB(2); qB3 = qB(3); qB4 = qB(4);
            CB11 = qB1^2 + qB2^2 - qB3^2 - qB4^2; CB12 = 2*(qB2*qB3 - qB1*qB4); CB13 = 2*(qB2*qB4 + qB1*qB3);
            CB21 = 2*(qB2*qB3 + qB1*qB4); CB22 = qB1^2 - qB2^2 + qB3^2 - qB4^2; CB23 = 2*(qB3*qB4 - qB1*qB2);
            CB31 = 2*(qB2*qB4 - qB1*qB3); CB32 = 2*(qB3*qB4 + qB1*qB2); CB33 = qB1^2 - qB2^2 - qB3^2 + qB4^2;
            theta_X(j) = atan2(C32, C33);
            if theta_X(j) < 0
                theta_X(j) = theta_X(j) + 2*pi;
            end
            theta_Y(j) = asin(-1*C31);
            if theta_Y(j) < 0
                theta_Y(j) = theta_Y(j) + 2*pi;
            end
            theta_Z(j) = atan2(C21, C11);
            if theta_Z(j) < 0
                theta_Z(j) = theta_Z(j) + 2*pi;
            end
            
            thetaB_X(j) = atan2(CB32, CB33);
%             if thetaB_X(j) < 0
%                 thetaB_X(j) = thetaB_X(j) + 2*pi;
%             end
            thetaB_Y(j) = asin(-1*CB31);
%             if thetaB_Y(j) < 0
%                 thetaB_Y(j) = thetaB_Y(j) + 2*pi;
%             end
            thetaB_Z(j) = atan2(CB21, CB11);
%             if thetaB_Z(j) < 0
%                 thetaB_Z(j) = thetaB_Z(j) + 2*pi;
%             end 
        end
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).calibration(i).euler.thetaX = theta_X;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).calibration(i).euler.thetaY = theta_Y;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).calibration(i).euler.thetaZ = theta_Z;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).calibration(i).eulerB.thetaX = thetaB_Z;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).calibration(i).eulerB.thetaY = -thetaB_Y;
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).calibration(i).eulerB.thetaZ = -thetaB_X;
    end
end