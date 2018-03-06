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

rawdata = csvread('ir_output.csv');

for(i=1:length(rawdata)/50)
    m = (i-1)*50 + 1;
    avg_data(i,1) = mean(rawdata(m:m+49));
    std_dev(i,1) = std(rawdata(m:m+49));
end

t = 0:length(avg_data);

% determine nth order Recursive LLS curve fit
%%% xhat -> nth order coefficient estimates
%%% yLS  -> nth order Recursive Least Squares curve fit data points
%%% RMS  -> Root Mean Square Error
[xhat1,yLS1,RMS1] = RecursiveLS_CurveFit(t,avg_data,1);
[xhat2,yLS2,RMS2] = RecursiveLS_CurveFit(t,avg_data,2);
[xhat3,yLS3,RMS3] = RecursiveLS_CurveFit(t,avg_data,3);
[xhat4,yLS4,RMS4] = RecursiveLS_CurveFit(t,avg_data,4);


t = (t+1)*50;

% plot results of Recursive LLS curve fit
figure(1)
scatter(1:length(rawdata),rawdata,4,'filled','r'); grid; hold on
errorbar(t(1:end-1)-25,avg_data,std_dev,'b'); hold off;
xlabel('Index','Interpreter','latex');
ylabel('IR Reading [-]','Interpreter','latex');
h = title('IR Sensor Characterization'); set(h,'Interpreter','latex')
legend('Raw IR data','Averaged Data Points','Location','NW')

figure(2);
scatter(1:length(rawdata),rawdata,4,'filled','r'); grid; hold on
errorbar(t(1:end-1)-25,avg_data,std_dev,'b');
plot(t-25,yLS1,'-o','MarkerSize',4);
plot(t-25,yLS2,'-o','MarkerSize',4);
plot(t-25,yLS3,'-o','MarkerSize',4);
plot(t-25,yLS4,'-o','MarkerSize',4); hold off
xlabel('Index','Interpreter','latex');
ylabel('IR Reading [-]','Interpreter','latex');
h = title('IR Sensor Characterization'); set(h,'Interpreter','latex')
ylim([0 3.5]);
legend({'Raw IR data','Averaged Data Points','Linear Fit',...
    'Quadratic Fit', 'Cubic Fit', 'Quartic Fit'},'Location','NW')

