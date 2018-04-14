function [IMU] = h5fdataConverter
% This function was written by Stephen Cain (smcain@umich.edu).
% Modified by Qisen Cheng (qisench@umich.edu)
% Updated 12/21/16
% This function imports time, acceleration, angular velocity, magnetometer,
% and quaternion orientation data from a APDM hdf5 data file. If a sensor
% is missing a data point, this function uses linear interpolation to fill
% in this point for a, w, and m, and uses SLERP to fill in this point for
% q. This new version also creates a field called 'button' that contains
% both the index of button pushes as well as the time of button pushes.

% Locate file
[filename, pathname] = uigetfile('*.h5','Select an APDM .h5 file');
cd(pathname)

% Read sensor data
caseIdList = hdf5read(filename,'/','CaseIdList');
monitorList = hdf5read(filename,'/','MonitorLabelList');

fileInfo = hdf5info(filename);
% assignin('base', 'filename', filename)
% assignin('base', 'caseIdList', caseIdList)
% assignin('base', 'monitorList', monitorList)
assignin('base', 'fileInfo', fileInfo)

%force monitorlist and caseIDList to be sacrum
% setData(caseIdList,'sacrum');
setLength(caseIdList, 16.0);
setName(caseIdList,'');
setPadding(caseIdList,'nullterm');
setData(monitorList,'sacrum');
setLength(monitorList, 16.0);
setName(monitorList,'');
setPadding(monitorList,'nullterm');

vectorlengths = zeros(length(caseIdList),1);

for jj = 1:length(caseIdList)
    
%     assignin('base', 'jj', jj)
    groupName = caseIdList(jj).data;
    sensorName = [groupName(1:2) groupName(4:9)];
    time.(sensorName) = double(hdf5read(filename, [groupName '/Time']));
    
    assignin('base', 'groupName', groupName)
    assignin('base','fieldName',monitorList(jj).data)
    
    IMU.(monitorList(jj).data).a = hdf5read(filename, [groupName '/Calibrated/Accelerometers'])';
    IMU.(monitorList(jj).data).w = hdf5read(filename, [groupName '/Calibrated/Gyroscopes'])';
    IMU.(monitorList(jj).data).m = hdf5read(filename, [groupName '/Calibrated/Magnetometers'])';
    IMU.(monitorList(jj).data).q = hdf5read(filename, [groupName '/Calibrated/Orientation'])';
    IMU.(monitorList(jj).data).time = (time.(sensorName) - time.(sensorName)(1))/(10^6); % convert time to seconds starting at zero 
    if strcmpi(monitorList(jj).data,'trigger')
        IMU.(monitorList(jj).data).button = hdf5read(filename, [groupName '/ButtonStatus']);
    else
    end
    
    % Record time vector length for each sensor
    vectorlengths(jj) = length(time.(sensorName));
end

% Identify if there are different numbers of data points
[Cu,ia,~] = unique(vectorlengths);
count = hist(vectorlengths,Cu);
[~,I] = max(count);

assignin('base','leng',vectorlengths(1))
assignin('base', 'Cu', Cu)
for kk = 1:length(caseIdList)
    if length(Cu) == 1
    elseif vectorlengths(kk) ~= Cu(I)
        index = 1:Cu(I);
        
        % Linear interpolate a, w, and m data to match majority of other
        % sensors data points
        IMU.(monitorList(kk).data).a = interp1(IMU.(monitorList(kk).data).time,IMU.(monitorList(kk).data).a,IMU.(monitorList(ia(I)).data).time,'linear','extrap');
        IMU.(monitorList(kk).data).w = interp1(IMU.(monitorList(kk).data).time,IMU.(monitorList(kk).data).w,IMU.(monitorList(ia(I)).data).time,'linear','extrap');
        IMU.(monitorList(kk).data).m = interp1(IMU.(monitorList(kk).data).time,IMU.(monitorList(kk).data).m,IMU.(monitorList(ia(I)).data).time,'linear','extrap');
          
        % Identify location of missed data point
        if vectorlengths(kk) < Cu(I)
            [match, ~] = ismember(IMU.(monitorList(ia(I)).data).time,IMU.(monitorList(kk).data).time);
            index_create = index(~match);
            
            % Use SLERP to interpolate quaternion
            if length(index_create) == 1
                t1 = IMU.(monitorList(ia(I)).data).time(index_create - 1);
                t2 = IMU.(monitorList(ia(I)).data).time(index_create + 1);
                tc = IMU.(monitorList(ia(I)).data).time(index_create);

                t2 = t2 - t1;
                tc = tc - t1;
                tc = tc/t2;

                q1 = IMU.(monitorList(kk).data).q(index_create - 1,:);
                q2 = IMU.(monitorList(kk).data).q(index_create,:);
                qc = slerp(q1, q2, tc);
                
                new_q = [IMU.(monitorList(kk).data).q(1:(index_create - 1),:); qc; IMU.(monitorList(kk).data).q(index_create:end,:)];
                
                % Correct missing data
                IMU.(monitorList(kk).data).q = new_q;
                
                if strcmpi(monitorList(kk).data,'trigger')
                    if IMU.trigger.button(index_create - 1) == IMU.trigger.button(index_create)
                        new_button = [IMU.trigger.button(1:(index_create - 1)); IMU.trigger.button(index_create); IMU.trigger.button(index_create:end)];
                        IMU.trigger.button = new_button;
                    else
                        break
                    end
                else
                end
                
            else
                break
            end          
        else
            break
        end
    else    
    end
end

if length(Cu) == 1
    IMU.time = IMU.(monitorList(1).data).time;
else
    IMU.time = IMU.(monitorList(ia(I)).data).time;
end

for bb = 1:length(caseIdList)
    
    % Delete repeated time vectors (only one is now needed)
    IMU.(monitorList(bb).data) = rmfield(IMU.(monitorList(bb).data),'time');
    
    % Create vectors of button push indexes and times
    if strcmpi(monitorList(bb).data,'trigger')
       
        % Identify locations of button pushes
        button_push = [0; diff(double(IMU.trigger.button))];
        push_index = zeros(sum(diff(IMU.trigger.button)),1);
        aa = 1;
        for qq = 1:length(button_push)
            if button_push(qq) == 1
                push_index(aa) = qq;
                aa = aa + 1;
            else
            end
        end
        
        IMU.button.push_index = push_index;
        IMU.button.push_time = IMU.time(push_index);
    else
    end
    
end

if isfield(IMU,'trigger')
    % Trigger data is no longer needed
    IMU = rmfield(IMU,'trigger');
else
end

% Save data
save(filename(1:(end-3)), 'IMU', '-v7.3')

end

function [qm] = slerp(qi, qn, t)
%       Sagi Dalyot %

%       This routine aims for calculating a unit quaternion,  describing a rotation matrix,
%       which lies between two known unit quaternions - q1 and q2,
%       using a spherical linear interpolation - Slerp.
%       Slerp follow the shortest great arc on a unit sphere,
%       hence, the shortest possible interpolation path.
%       Consequently, Slerp has constant angular velocity, 
%       so it is the optimal interpolation curve between two rotations.
%       (first published by Sheomake K., 1985 - Animating Rotation with Quaternion Curves)

%       end of file ->  explnation of rotation matrix and quaternions

%       in general:
%       slerp(q1, q2, t) = q1*(sin(1-t)*teta)/sin(t) + q2*(sin(t*teta))/sin(teta)
%       where teta is the angle between the two unit quaternions,
%       and t is between [0,1]

%       two border cases will be delt:
%       1: where q1 = q2 (or close by eps)
%       2: where q1 = -q2 (angle between unit quaternions is 180 degrees).
%       in general, if q1=q2 then Slerp(q; q; t) == q


%       where qi=[w1 x1 y1 z1] - start unit quaternions
%                      qn=[w2 x2 y2 z2] - end unit quaternions
%                      t=[0 to 1]
%                      eps=threshold value

if t==0 % saving calculation time -> where qm=qi
    qm=qi;
    
elseif t==1 % saving calculation time -> where qm=qn
    qm=qn;
    
else

    C=dot(qi,qn);                  % Calculating the angle beteen the unit quaternions by dot product

    teta=acos(C);

%         if (1 - C) <= eps % if angle teta is close by epsilon to 0 degrees -> calculate by linear interpolation
%             qm=qi*(1-t)+qn*t; % avoiding divisions by number close to 0
% 
%         elseif (1 + C) <= eps % when teta is close by epsilon to 180 degrees the result is undefined -> no shortest direction to rotate
%             q2(1) = qi(4); q2(2) = -qi(3); q2(3)= qi(2); q2(4) = -qi(1); % rotating one of the unit quaternions by 90 degrees -> q2
%             qm=qi*(sin((1-t)*(pi/2)))+q2*sin(t*(pi/2));
% 
%         else
            qm=qi*(sin((1-t)*teta))/sin(teta)+qn*sin(t*teta)/sin(teta);
%         end
end
end