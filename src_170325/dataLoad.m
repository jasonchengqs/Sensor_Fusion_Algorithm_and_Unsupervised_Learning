% This function was written by Qisen Cheng (qisench@umich.edu).
% Updated 1/20/17
% This function loads the latest imu mat file into the workspace

function [imu] = dataLoad
files = dir('imu_*.mat');

for i=1:length(files)
    temp = datetime(files(i).date);
    if (i == 1) 
        last = temp;
        fileId = 1;
    end
    if (i > 1) && (last < temp)
        last = temp;
        fileId = i;
    end
end
data = load(files(fileId).name);
name = fieldnames(data);
imu = data.(name{1,1});