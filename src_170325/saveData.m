function saveData(imu)
data = imu;
updateMark = datestr(datetime('today'));
fileName = strcat('imu_',updateMark,'.mat');
save(fileName, 'data');