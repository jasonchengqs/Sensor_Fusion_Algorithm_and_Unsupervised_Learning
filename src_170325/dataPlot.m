function dataPlot(imu, combo, target, session, plotCmd)
%% Set the default colors for plotting
co = [0         0    1.0000;
     1.0000         0         0;
    0.9290    0.6940    0.1250;
         0    0.7500    0.7500;
    0.7500         0    0.7500;
    0.7500    0.7500         0;
    0.2500    0.2500    0.2500];
set(groot,'defaultAxesColorOrder',co)

%% data plot
% target = 'Elius';
% combo = 'combo_1';
curData = imu.(target).data.(combo);

if ~isempty(strfind(plotCmd, 'raw_sensor'))
    for p = 1:2
        if p == 1
            figure ('Name',strcat(target,'_raw accel in sensor frame'));
        else if p == 2
                figure ('Name',strcat(target,'_raw gyro in sensor frame'))
            end
        end
        for j = 1:length(curData.(session))
            subplot(3,ceil(length(curData.(session))/3),j);
            hold on
            if (p == 1)
                plot(curData.(session)(j).sensor_frame.accel.X);
                plot(curData.(session)(j).sensor_frame.accel.Y);
                plot(curData.(session)(j).sensor_frame.accel.Z);
                legend('acc-X','acc-Y','acc-Z')
            else
                plot(curData.(session)(j).sensor_frame.gyro.X);
                plot(curData.(session)(j).sensor_frame.gyro.Y);
                plot(curData.(session)(j).sensor_frame.gyro.Z);
                legend('gyro-X','gyro-Y','gyro-Z')
            end
        end
    end
end

if ~isempty(strfind(plotCmd, 'raw_navi'))
    for p = 1:2
        if p == 1
            figure ('Name',strcat(target,'_converted accel in navi-frame'));
        else if p == 2
                figure ('Name',strcat(target,'_converted gyro in navi-frame'))
            end
        end
        for j = 1:length(curData.(session))
            subplot(3,ceil(length(curData.(session))/3),j);
            hold on
            if (p == 1)
                plot(curData.(session)(j).navi_frame.accel.X);
                plot(curData.(session)(j).navi_frame.accel.Y);
                plot(curData.(session)(j).navi_frame.accel.Z);
                legend('acc-X','acc-Y','acc-Z')
            else
                plot(curData.(session)(j).navi_frame.gyro.X);
                plot(curData.(session)(j).navi_frame.gyro.Y);
                plot(curData.(session)(j).navi_frame.gyro.Z);
                legend('gyro-X','gyro-Y','gyro-Z')
            end
        end
    end
end

if ~isempty(strfind(plotCmd, 'integ_angles_sensor'))
    figure('Name',strcat(target,'_angles (in sensor frame) from integration'))
    for j = 1:length(curData.(session))
        subplot(3,ceil(length(curData.(session))/3),j);
        hold on
        plot(curData.(session)(j).sensor_frame.inteAngles.X/pi*180);
        plot(curData.(session)(j).sensor_frame.inteAngles.Y/pi*180);
        plot(curData.(session)(j).sensor_frame.inteAngles.Z/pi*180);
%         plot(curData.(opt)(j).cmplt.thetaX/pi*180 - mean(curData.(opt)(j).cmplt.thetaX/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaY/pi*180 - mean(curData.(opt)(j).cmplt.thetaY/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaZ/pi*180 - mean(curData.(opt)(j).cmplt.thetaZ/pi*180));
        legend('theta-X','theta-Y','theta-Z')
        xlabel('t')
        ylabel('deg')
    end
end

if ~isempty(strfind(plotCmd, 'integ_angles_navi'))
    figure('Name',strcat(target,'_angles (in navigation frame) from integration'))
    for j = 1:length(curData.(session))
        subplot(3,ceil(length(curData.(session))/3),j);
        hold on
        plot(curData.(session)(j).navi_frame.inteAngles.X/pi*180);
        plot(curData.(session)(j).navi_frame.inteAngles.Y/pi*180);
        plot(curData.(session)(j).navi_frame.inteAngles.Z/pi*180);
%         plot(curData.(opt)(j).cmplt.thetaX/pi*180 - mean(curData.(opt)(j).cmplt.thetaX/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaY/pi*180 - mean(curData.(opt)(j).cmplt.thetaY/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaZ/pi*180 - mean(curData.(opt)(j).cmplt.thetaZ/pi*180));
        legend('theta-X','theta-Y','theta-Z')
        xlabel('t')
        ylabel('deg')
    end
end

if ~isempty(strfind(plotCmd, 'noG_accel_sensor'))
    figure('Name',strcat(target,'_gravity compensated acceleration (in sensor frame)'))
    for j = 1:length(curData.(session))
        subplot(3,ceil(length(curData.(session))/3),j);
        hold on
        plot(curData.(session)(j).sensor_frame.noG_accel.X);
        plot(curData.(session)(j).sensor_frame.noG_accel.Y);
        plot(curData.(session)(j).sensor_frame.noG_accel.Z);
        legend('acc-X','acc-Y','acc-Z')
        xlabel('t')
        ylabel('m/s^2')
    end
end

if ~isempty(strfind(plotCmd, 'noG_accel_navi'))
    figure('Name',strcat(target,'_gravity compensated acceleration (in navigation frame)'))
    for j = 1:length(curData.(session))
        subplot(3,ceil(length(curData.(session))/3),j);
        hold on
        plot(curData.(session)(j).navi_frame.noG_accel.X);
        plot(curData.(session)(j).navi_frame.noG_accel.Y);
        plot(curData.(session)(j).navi_frame.noG_accel.Z);
        legend('acc-X','acc-Y','acc-Z')
        xlabel('t')
        ylabel('m/s^2')
    end
end

if ~isempty(strfind(plotCmd, 'cmplt_angles_sensor'))
    figure('Name',strcat(target,'_angles (in sensor frame) from complementary filter'))
    for j = 1:length(curData.(session))
        subplot(3,ceil(length(curData.(session))/3),j);
        hold on
        plot(curData.(session)(j).sensor_frame.cmplt_angle.X/pi*180);
        plot(curData.(session)(j).sensor_frame.cmplt_angle.Y/pi*180);
        plot(curData.(session)(j).sensor_frame.cmplt_angle.Z/pi*180);
%         plot(curData.(opt)(j).cmplt.thetaX/pi*180 - mean(curData.(opt)(j).cmplt.thetaX/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaY/pi*180 - mean(curData.(opt)(j).cmplt.thetaY/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaZ/pi*180 - mean(curData.(opt)(j).cmplt.thetaZ/pi*180));
        legend('theta-X','theta-Y','theta-Z')
        xlabel('t')
        ylabel('deg')
    end
end

if ~isempty(strfind(plotCmd, 'cmplt_angles_navi'))
    figure('Name',strcat(target,'_angles (in navi frame) from complementary filter'))
    for j = 1:length(curData.(session))
        subplot(3,ceil(length(curData.(session))/3),j);
        hold on
        plot(curData.(session)(j).navi_frame.cmplt_angle.X/pi*180);
        plot(curData.(session)(j).navi_frame.cmplt_angle.Y/pi*180);
        plot(curData.(session)(j).navi_frame.cmplt_angle.Z/pi*180);
%         plot(curData.(opt)(j).cmplt.thetaX/pi*180 - mean(curData.(opt)(j).cmplt.thetaX/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaY/pi*180 - mean(curData.(opt)(j).cmplt.thetaY/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaZ/pi*180 - mean(curData.(opt)(j).cmplt.thetaZ/pi*180));
        legend('theta-X','theta-Y','theta-Z')
        xlabel('t')
        ylabel('deg')
    end
end

if ~isempty(strfind(plotCmd, 'kalman_angles_navi'))
    figure('Name',strcat(target,'_angles (in navi frame) from kalman filter'))
    for j = 1:length(curData.(session))
        subplot(3,ceil(length(curData.(session))/3),j);
        hold on
        plot(curData.(session)(j).navi_frame.kalman_angle.X/pi*180);
        plot(curData.(session)(j).navi_frame.kalman_angle.Y/pi*180);
        plot(curData.(session)(j).navi_frame.kalman_angle.Z/pi*180);
%         plot(curData.(opt)(j).cmplt.thetaX/pi*180 - mean(curData.(opt)(j).cmplt.thetaX/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaY/pi*180 - mean(curData.(opt)(j).cmplt.thetaY/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaZ/pi*180 - mean(curData.(opt)(j).cmplt.thetaZ/pi*180));
        legend('theta-X','theta-Y','theta-Z')
        xlabel('t')
        ylabel('deg')
    end
end

if ~isempty(strfind(plotCmd, 'kalman_angles_sensor'))
    figure('Name',strcat(target,'_angles (in sensor frame) from kalman filter'))
    for j = 1:length(curData.(session))
        subplot(3,ceil(length(curData.(session))/3),j);
        hold on
        plot(curData.(session)(j).sensor_frame.kalman_angle.X/pi*180);
        plot(curData.(session)(j).sensor_frame.kalman_angle.Y/pi*180);
        plot(curData.(session)(j).sensor_frame.kalman_angle.Z/pi*180);
%         plot(curData.(opt)(j).cmplt.thetaX/pi*180 - mean(curData.(opt)(j).cmplt.thetaX/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaY/pi*180 - mean(curData.(opt)(j).cmplt.thetaY/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaZ/pi*180 - mean(curData.(opt)(j).cmplt.thetaZ/pi*180));
        legend('theta-X','theta-Y','theta-Z')
        xlabel('t')
        ylabel('deg')
    end
end

if ~isempty(strfind(plotCmd, 'madgwick_angles_navi'))
    figure('Name',strcat(target,'_angles (in navi frame) from madgwick algorithm'))
    for j = 1:length(curData.(session))
        subplot(3,ceil(length(curData.(session))/3),j);
        hold on
        plot(curData.(session)(j).navi_frame.madgwick_angle.X/pi*180);
        plot(curData.(session)(j).navi_frame.madgwick_angle.Y/pi*180);
        plot(curData.(session)(j).navi_frame.madgwick_angle.Z/pi*180);
%         plot(curData.(opt)(j).cmplt.thetaX/pi*180 - mean(curData.(opt)(j).cmplt.thetaX/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaY/pi*180 - mean(curData.(opt)(j).cmplt.thetaY/pi*180));
%         plot(curData.(opt)(j).cmplt.thetaZ/pi*180 - mean(curData.(opt)(j).cmplt.thetaZ/pi*180));
        legend('theta-X','theta-Y','theta-Z')
        xlabel('t')
        ylabel('deg')
    end
end

if ~isempty(strfind(plotCmd, 'raw_orientation_navi'))
    figure('Name',strcat(target,'_orientation (in navi frame) from raw quarternion'))
    for j = 1:length(curData.(session))
        subplot(3,ceil(length(curData.(session))/3),j);
        hold on
        plot(curData.(session)(j).navi_frame.raw_angle.X/pi*180);
        plot(curData.(session)(j).navi_frame.raw_angle.Y/pi*180);
        plot(curData.(session)(j).navi_frame.raw_angle.Z/pi*180);
        legend('theta-X','theta-Y','theta-Z')
        xlabel('t')
        ylabel('deg')
    end
end

%%
% if ~isempty(strfind(plotCmd, 'cmplt_deltaAngles_sensor'))
%         figure('Name',strcat(target,'_delta angles (in sensor frame) from complementary filter'))
%     for j = 1:length(curData.(session))
%         dAngleX = zeros(size(curData.(session)(j).cmplt.thetaX));
%         dAngleY = zeros(size(curData.(session)(j).cmplt.thetaY));
%         dAngleZ = zeros(size(curData.(session)(j).cmplt.thetaZ));
%         for t = 2:length(curData.(session)(j).cmplt.thetaX)
%             dAngleX(t) = curData.(session)(j).cmplt.thetaX(t)/pi*180 - ...
%                 curData.(session)(j).cmplt.thetaX(t-1)/pi*180;
%             dAngleY(t) = curData.(session)(j).cmplt.thetaY(t)/pi*180 - ...
%                 curData.(session)(j).cmplt.thetaY(t-1)/pi*180;
%             dAngleZ(t) = curData.(session)(j).cmplt.thetaZ(t)/pi*180 - ...
%                 curData.(session)(j).cmplt.thetaZ(t-1)/pi*180;
%         end
%         subplot(3,ceil(length(curData.(session))/3),j);
%         hold on
%         plot(dAngleX);
%         plot(dAngleY);
%         plot(dAngleZ);
%         legend('theta-X','theta-Y','theta-Z')
%         xlabel('t')
%         ylabel('deg')
%     end
% end
% 
% 
% 
% if ~isempty(strfind(plotCmd, 'quaternion_angle_navi'))
%     figure('Name',strcat(target,'_angles (in navi frame) from quarternion'))
%     for j = 1:length(curData.(session))
%         subplot(3,ceil(length(curData.(session))/3),j);
%         hold on
%         plot(curData.(session)(j).euler.thetaX/pi*180-150);
%         plot(curData.(session)(j).euler.thetaY/pi*180-280);
%         plot(curData.(session)(j).euler.thetaZ/pi*180-220);
%         legend('theta-X','theta-Y','theta-Z')
%         xlabel('t')
%         ylabel('deg')
%     end
% end
% 
% if ~isempty(strfind(plotCmd, 'inte_velocity_navi'))
%     figure('Name',strcat(target,'_velocity (in navigation frame) from integration of acc in Navi'))
%     for j = 1:length(curData.(session))
%         subplot(3,ceil(length(curData.(session))/3),j);
%         hold on
%         plot(curData.(session)(j).navigation.velocity.vX);
%         plot(curData.(session)(j).navigation.velocity.vY);
%         plot(curData.(session)(j).navigation.velocity.vZ);
%         legend('v-X','v-Y','v-Z')
%         xlabel('t')
%         ylabel('m/s')
%     end
% end
% 
% 
% if ~isempty(strfind(plotCmd, 'cmplt_angle_navi'))
%     figure('Name',strcat(target,'angles (in Navigation frame) from complementary filter'))
%     for j = 1:length(curData.(session))
%         subplot(3,ceil(length(curData.(session))/3),j);
%         hold on
%         plot(curData.(session)(j).navigation.cmplt.thetaX/pi*180);
%         plot(curData.(session)(j).navigation.cmplt.thetaY/pi*180);
%         plot(curData.(session)(j).navigation.cmplt.thetaZ/pi*180);
%         legend('theta-X','theta-Y','theta-Z')
%         xlabel('t')
%         ylabel('deg')
%     end
% end
% 
% if ~isempty(strfind(plotCmd, 'position_navi'))
%     figure('Name',strcat(target,'_position (in navi frame) from integration of acc'))
%     for j = 1:length(curData.(session))
%         subplot(3,ceil(length(curData.(session))/3),j);
%         hold on
%         plot(curData.(session)(j).navigation.position.vX);
%         plot(curData.(session)(j).navigation.position.vY);
%         plot(curData.(session)(j).navigation.position.vZ);
%         legend('p-X','p-Y','p-Z')
%         xlabel('t')
%         ylabel('m')
%     end
% end
% 
% if ~isempty(strfind(plotCmd, 'quaternion_angle_sensor'))
%     figure('Name',strcat(target,'_angles (in sensor frame) from quarternion'))
%     for j = 1:length(curData.(session))
%         subplot(3,ceil(length(curData.(session))/3),j);
%         hold on
%         plot(curData.(session)(j).eulerB.thetaX/pi*180);
%         plot(curData.(session)(j).eulerB.thetaY/pi*180);
%         plot(curData.(session)(j).eulerB.thetaZ/pi*180);
%         legend('theta-X','theta-Y','theta-Z')
%         xlabel('t')
%         ylabel('deg')
%     end
% end
end