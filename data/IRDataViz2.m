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

% NOTE: Difference between doorway1_1 and doorway1_1_2 is that for
% doorway1_1 outlier removal was disabled. It was re-enabled for
% doorway1_1_2 and for all other data sets

doorway1 = csvread('ir_course_data_doorway1_1_2.csv');
doorway2 = csvread('ir_course_data_doorway2_1.csv');
doorway3 = csvread('ir_course_data_doorway3_1.csv');
corner1 = csvread('ir_course_data_corner1_1.csv');
corner2 = csvread('ir_course_data_corner2_1.csv');
blockedwindow = csvread('ir_course_data_blockedwindow_1.csv');
unblockedwindow = csvread('ir_course_data_unblockedwindow_1.csv');
hallway = csvread('ir_course_data_hallway_1.csv');

t1 = 1:length(doorway1);
t2 = 1:length(doorway2);
t3 = 1:length(doorway3);
t4 = 1:length(corner1);
t5 = 1:length(corner2);
t6 = 1:length(blockedwindow);
t7 = 1:length(unblockedwindow);
t8 = 1:length(hallway);

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

err_b6 = blockedwindow(:,1);
err_t6 = blockedwindow(:,2);
diff_b6 = blockedwindow(:,3);
diff_t6 = blockedwindow(:,4);

err_b7 = unblockedwindow(:,1);
err_t7 = unblockedwindow(:,2);
diff_b7 = unblockedwindow(:,3);
diff_t7 = unblockedwindow(:,4);

err_b8 = hallway(:,1);
err_t8 = hallway(:,2);
diff_b8 = hallway(:,3);
diff_t8 = hallway(:,4);

figure(1)
plot(t1,diff_b1,t1,diff_t1,t1,err_b1,t1,err_t1)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Doorway 1'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(2)
plot(t2,diff_b2,t2,diff_t2,t2,err_b2,t2,err_t2)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Doorway 2'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(3)
plot(t3,diff_b3,t3,diff_t3,t3,err_b3,t3,err_t3)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Doorway 3'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(4)
plot(t4,diff_b4,t4,diff_t4,t4,err_b4,t4,err_t4)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Corner 1'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(5)
plot(t5,diff_b5,t5,diff_t5,t5,err_b5,t5,err_t5)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Corner 2'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(6)
plot(t6,diff_b6,t6,diff_t6,t6,err_b6,t6,err_t6)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Blocked Window/Door'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(7)
plot(t7,diff_b7,t7,diff_t7,t7,err_b7,t7,err_t7)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Unblocked Window/Door'); set(h,'Interpreter','latex')
legend('diff_b','diff_t','err_b','err_t','Location','NW')

figure(8)
plot(t8,diff_b8,t8,diff_t8,t8,err_b8,t8,err_t8)
xlabel('Index','Interpreter','latex');
ylabel('Data Value [cm]','Interpreter','latex');
h = title('Hallway - No Obstacless'); set(h,'Interpreter','latex')
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
