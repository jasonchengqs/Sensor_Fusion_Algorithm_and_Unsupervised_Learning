function [imu] = velocity(imu,comboId, opt)
% opt = 'calibration';
% comboId = 1;
names = fieldnames(imu);
f = 128;
dt = 1/f;
g = 9.8;
for k = 1:length(names)
    target = names{k};
    combo = strcat('combo_',num2str(comboId));
    curData = imu.(target).data.(combo);
    for i = 1:length(curData.(opt))
        vX = zeros(length(curData.(opt)(i).navigation.accNavi.X)+1,1);
        vY = zeros(length(curData.(opt)(i).navigation.accNavi.Y)+1,1);
        vZ = zeros(length(curData.(opt)(i).navigation.accNavi.Z)+1,1);
        biasX = 0;
        biasY = 0;
        biasZ = 0;
%         biasX = 0.5*(min(curData.(opt)(i).navigation.accNavi.X)+max(curData.(opt)(i).navigation.accNavi.X));
%         biasY = 0.5*(min(curData.(opt)(i).navigation.accNavi.Y)+max(curData.(opt)(i).navigation.accNavi.Y));
%         biasZ = 0.5*(min(curData.(opt)(i).navigation.accNavi.Z)+max(curData.(opt)(i).navigation.accNavi.Z));
%         biasX = mean(curData.(opt)(i).navigation.accNavi.X);
%         biasY = mean(curData.(opt)(i).navigation.accNavi.Y);
%         biasZ = mean(curData.(opt)(i).navigation.accNavi.Z);
        for j = 2:length(curData.(opt)(i).quarternion.q1)
            accX = curData.(opt)(i).navigation.accNavi.X(j-1);
            accY = curData.(opt)(i).navigation.accNavi.Y(j-1);
            accZ = curData.(opt)(i).navigation.accNavi.Z(j-1)-g;
            vX(j) = vX(j-1) + (-1) * (accX-biasX)*dt;
            vY(j) = vY(j-1) + (-1) * (accY-biasY)*dt;
            vZ(j) = vZ(j-1) + (-1) * (accZ-biasZ)*dt;
        end
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(opt)(i).navigation.velocity.vX = detrend(vX);
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(opt)(i).navigation.velocity.vY = detrend(vY);
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(opt)(i).navigation.velocity.vZ = detrend(vZ);
    end
end