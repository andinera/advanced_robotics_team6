%% Header

% Group:        Advanced Robotics Team 6 
% File:         IR_Sensor_Characterization.m
% Author:       Carl Stahoviak
% Date:         03/06/2018

clear;
clc;
close ALL;

format compact

%% IR Sensor Characterization

rawdata = csvread('IR_out_031718.csv');

d = rawdata(:,1);
ir_top = rawdata(:,2);
ir_bottom = rawdata(:,3);

% ir_top = 1./ir_top;
% ir_bottom = 1./ir_bottom;

for(i=1:length(rawdata)/50)
    m = (i-1)*50 + 1;
    data_avg(i,1) = mean(ir_bottom(m:m+49));
    std_dev(i,1) = std(ir_bottom(m:m+49));
    d_avg(i,1) = mean(d(m:m+49));
end

figure(1)
plot(data_avg,d_avg,'.')

p1 = polyfit(data_avg(4:end),d_avg(4:end),1);
p2 = polyfit(data_avg(4:end),d_avg(4:end),2);
p3 = polyfit(data_avg(4:end),d_avg(4:end),3);
p4 = polyfit(data_avg(4:end),d_avg(4:end),4);

[y1,rms1] = createPolynomial(data_avg(4:end),d_avg(4:end),p1);
[y2,rms2] = createPolynomial(data_avg(4:end),d_avg(4:end),p2);
[y3,rms3] = createPolynomial(data_avg(4:end),d_avg(4:end),p3);
[y4,rms4] = createPolynomial(data_avg(4:end),d_avg(4:end),p3);

p1 = [p1'; zeros(3,1)];
p2 = [p2'; zeros(2,1)];
p3 = [p3'; zeros(1,1)];
p4 = p4';

M = [p1 p2 p3 p4];
csvwrite('IR_fit_coeff_031718.csv',M)

% plot results of Recursive LLS curve fit
figure(2)
scatter(ir_bottom,d,4,'filled','r'); grid; hold on
errorbar(data_avg,d_avg,std_dev,'horizontal','b'); hold off;
xlabel('Raw IR Data','Interpreter','latex');
ylabel('True Position [cm]','Interpreter','latex');
h = title('IR Sensor Characterization'); set(h,'Interpreter','latex')
legend('Raw IR data','Averaged Data Points','Location','NW')

figure(3);
scatter(ir_bottom,d,4,'filled','r'); grid; hold on
errorbar(data_avg,d_avg,std_dev,'horizontal','b');
plot(data_avg(4:end),y1,'-o','MarkerSize',4);
plot(data_avg(4:end),y2,'-o','MarkerSize',4);
plot(data_avg(4:end),y3,'-o','MarkerSize',4);
plot(data_avg(4:end),y4,'-o','MarkerSize',4); hold off
xlabel('Index','Interpreter','latex');
ylabel('IR Reading [-]','Interpreter','latex');
h = title('IR Sensor Characterization'); set(h,'Interpreter','latex')
% ylim([0 3.5]);
legend({'Raw IR data','Averaged Data Points','Linear Fit',...
    'Quadratic Fit', 'Cubic Fit', 'Quartic Fit'},'Location','NW')

