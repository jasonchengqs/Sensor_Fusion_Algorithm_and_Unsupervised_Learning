function [imu] = position(imu,comboId, opt)
% opt = 'calibration';
% comboId = 1;
names = fieldnames(imu);
f = 128;
dt = 1/f;
for k = 1:length(names)
    target = names{k};
    combo = strcat('combo_',num2str(comboId));
    curData = imu.(target).data.(combo);
    for i = 1:length(curData.(opt))
        pX = zeros(length(curData.(opt)(i).navigation.velocity.vX),1);
        pY = zeros(length(curData.(opt)(i).navigation.velocity.vY),1);
        pZ = zeros(length(curData.(opt)(i).navigation.velocity.vZ),1);
        biasX = 0;
        biasY = 0;
        biasZ = 0;
%         biasX = 0.5*(min(curData.(opt)(i).navigation.accNavi.X)+max(curData.(opt)(i).navigation.accNavi.X));
%         biasY = 0.5*(min(curData.(opt)(i).navigation.accNavi.Y)+max(curData.(opt)(i).navigation.accNavi.Y));
%         biasZ = 0.5*(min(curData.(opt)(i).navigation.accNavi.Z)+max(curData.(opt)(i).navigation.accNavi.Z));
%         biasX = mean(curData.(opt)(i).navigation.accNavi.X);
%         biasY = mean(curData.(opt)(i).navigation.accNavi.Y);
%         biasZ = mean(curData.(opt)(i).navigation.accNavi.Z);
        for j = 2:length(curData.(opt)(i).navigation.velocity.vX)
            vX = curData.(opt)(i).navigation.velocity.vX(j-1);
            vY = curData.(opt)(i).navigation.velocity.vY(j-1);
            vZ = curData.(opt)(i).navigation.velocity.vZ(j-1);
            pX(j) = pX(j-1) + (vX-biasX)*dt;
            pY(j) = pY(j-1) + (vY-biasY)*dt;
            pZ(j) = pZ(j-1) + (vZ-biasZ)*dt;
        end
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(opt)(i).navigation.position.vX = detrend(pX);
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(opt)(i).navigation.position.vY = detrend(pY);
        imu.(names{k}).data.(strcat('combo_',num2str(comboId))).(opt)(i).navigation.position.vZ = detrend(pZ);
    end
end