%% Header

% Group:        Advanced Robotics Team 6 
% File:         IRDataViz.m
% Author:       Carl Stahoviak
% Date:         03/21/2018

clear;
clc;
close ALL;

format compact

%% IR Sensor Course Vizualization

% data = [ir_bottom_error, ir_top_error, ir_bottom_diff, ir_top_diff]

doorway1 = csvread('ir_course_data_doorway1.csv');
doorway2 = csvread('ir_course_data_doorway2.csv');
doorway3 = csvread('ir_course_data_doorway3.csv');
corner1 = csvread('ir_course_data_corner1.csv');
corner2 = csvread('ir_course_data_corner2.csv');
doorwindow = csvread('ir_course_data_doorwindow.csv');

t = 1:length(doorway1);
t2 = 1:length(corner1);

err_b1 = doorway1(:,1);
err_t1 = doorway1(:,2);
diff_b1 = doorway1(:,3);
diff_t1 = doorway1(:,4);

err_b2 = doorway2(:,1);
err_t2 = doorway2(:,2);
diff_b2 = doorway2(:,3);
diff_t2 = doorway2(:,4);

err_b3 = doorway3(:,1);
err_t3 = doorway3(:,2);
diff_b3 = doorway3(:,3);
diff_t3 = doorway3(:,4);

err_b4 = corner1(:,1);
err_t4 = corner1(:,2);
diff_b4 = corner1(:,3);
diff_t4 = corner1(:,4);

err_b5 = corner2(:,1);
err_t5 = corner2(:,2);
diff_b5 = corner2(:,3);
diff_t5 = corner2(:,4);

err_b6 = doorwindow(:,1);
err_t6 = doorwindow(:,2);
diff_b6 = doorwindow(:,3);
diff_t6 = doorwindow(:,4);

figure(1)
plot(t,diff_b1,t,diff_t1,t,err_b1,t,err_t1)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Doorway 1'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(2)
plot(t,diff_b2,t,diff_t2,t,err_b2,t,err_t2)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Doorway 2'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(3)
plot(t,diff_b3,t,diff_t3,t,err_b3,t,err_t3)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Doorway 3'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(4)
plot(t2,diff_b4,t2,diff_t4,t2,err_b4,t2,err_t4)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Corner 1'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(5)
plot(t,diff_b5,t,diff_t5,t,err_b5,t,err_t5)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Corner 2'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(6)
plot(t,diff_b6,t,diff_t6,t,err_b6,t,err_t6)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Door/Window'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

% figure(4)
% subplot(3,1,1); plot(t,diff_b3,t,diff_t3,t,err_b3,t,err_t3)
% ylabel('Data Value [cm]','Interpreter','latex');
% subplot(3,1,2); plot(t,diff_b2,t,diff_t2,t,err_b2,t,err_t2)
% ylabel('Data Value [cm]','Interpreter','latex');
% subplot(3,1,3); plot(t,diff_b3,t,diff_t3,t,err_b3,t,err_t3)
% xlabel('Index','Interpreter','latex');
% ylabel('Data Value [cm]','Interpreter','latex');
% legend('diff_b','diff_t','err_b','err_t','Location','NW')
