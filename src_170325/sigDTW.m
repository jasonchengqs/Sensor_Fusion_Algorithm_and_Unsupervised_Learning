function sigDTW(imu, f, config)
name_1 = config{1};
name_2 = config{2};
comboId = config{3};
session = config{4};
k = config{5};
frame = config{6};
process = config{7};
axis = config{8};
tail = 0.5*f;
dt = 1/f;
% read in signals
sig1 = imu.(name_1).data.(strcat('combo_',num2str(comboId))).(session)(k(1)).(frame).(process).(axis) * 180/pi;
sig2 = imu.(name_2).data.(strcat('combo_',num2str(comboId))).(session)(k(2)).(frame).(process).(axis) * 180/pi;

% estimate width of search window (in seconds)
minWindow = 5;
window = ceil(abs(length(sig1)-length(sig2))/f);
if window < minWindow
    window = minWindow;
end

diff_sig1 = diff(sig1)/dt; diff_sig2 = diff(sig2)/dt;
max_delay = 1/4*min(length(diff_sig1),length(diff_sig2));
delay = finddelay(diff_sig1(1:(end-tail)),diff_sig2,max_delay);
if delay >= 0
    sig1_delay = sig1;
    diff_sig1_delay = diff_sig1;
    sig2_delay = sig2(delay:end);
    diff_sig2_delay = diff_sig2(delay:end);
end
if delay < 0
    sig1_delay = sig1(-delay:end);
    diff_sig1_delay = diff_sig1(-delay:end);
    sig2_delay = sig2;
    diff_sig2_delay = diff_sig2;
end

if length(diff_sig1_delay) > length(diff_sig2_delay)
    [dc,i2c,i1c] = dtw(diff_sig2_delay(1:(end-tail)),diff_sig1_delay(1:(end-tail)),f*window,'absolute');
else
    [dc,i1c,i2c] = dtw(diff_sig1_delay(1:(end-tail)),diff_sig2_delay(1:(end-tail)),f*window,'absolute');
end

sep1 = 80;
sep2 = 3*sep1;
figure
ax1_1 = subplot(2,2,1);
hold on
plot(sig1+sep1,'.-','color','blue')
plot(sig2,'.-','color','red')
title('original signal')
legend([name_1 '--trial#' num2str(k(1))],[name_2 '--trial#' num2str(k(2))])
ax1_2 = subplot(2,2,2);
hold on
plot(sig1_delay(1:(end-tail))+sep1,'.-','color','blue')
plot(sig2_delay(1:(end-tail)),'.-','color','red')
title(['delay compensated signal -- delay:' num2str(delay)])
legend([name_1 '--trial#' num2str(k(1))],[name_2 '--trial#' num2str(k(2))])
ax1_3 = subplot(2,2,3);
hold on
plot(diff_sig1_delay(1:(end-tail))+sep2,'.-','color','blue')
plot(diff_sig2_delay(1:(end-tail)),'.-','color','red')
title(['delay compensated derivative -- ' 'Distance: ' num2str(dc)])
legend([name_1 '--trial#' num2str(k(1))],[name_2 '--trial#' num2str(k(2))])
ax1_4 = subplot(2,2,4);
plot(i1c,i2c,'.')
title('optimal path matrix')

figure
ax21 = subplot(2,2,1);
hold on
plot(sig1_delay(i1c)+sep1,'.-','color','blue')
plot(sig2_delay(i2c),'.-','color','red')
legend([name_1 '--trial#' num2str(k(1))],[name_2 '--trial#' num2str(k(2))])
title(['Aligned (stretched) signals after DTW ','search window=',num2str(window)])
ax22 = subplot(2,2,2);
hold on
plot(diff_sig1_delay(i1c)+sep2,'.-','color','blue')
plot(diff_sig2_delay(i2c),'.-','color','red')
legend([name_1 '--trial#' num2str(k(1))],[name_2 '--trial#' num2str(k(2))])
title(['stretched derivative signals after DTW ','search window=',num2str(window)])
ax23 = subplot(2,2,3:4);
hold on
for i = 1:length(i1c)
    plot([i1c(i),i2c(i)],[sig1_delay(i1c(i))+sep1,sig2_delay(i2c(i))],'k','color','black')
end
plot(sig1_delay+sep1,'.-','color','blue')
plot(sig2_delay,'.-','color','red')
legend([name_1 '--trial#' num2str(k(1))],[name_2 '--trial#' num2str(k(2))])