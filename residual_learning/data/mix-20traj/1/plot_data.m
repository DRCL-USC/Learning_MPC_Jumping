clear all; close all;
% 
h_init = 0.1434; % initial robot height

pos_imu = dlmread('pos.txt'); % no limits
save('pos.mat','pos_imu');
N = size(pos_imu,1); % number of data point 
N_full = 1500; % take 3s

data_f = dlmread('data_f.txt'); % no limits
save('data_f.mat','data_f');

data_fcmd = dlmread('data_fcmd.txt'); % no limits
save('data_fcmd.mat','data_fcmd');

optitrack = dlmread('optitrack.txt');
save('optitrack.mat','optitrack');
start_jump = 4000;
pos_mocap = optitrack(1:N,1:3); % order: x, y, z
quat_mocap = optitrack(1:N,4:7);

x_lim = N; z_lim = 1;
fl_idx = 800;
sc_idx = 500;

%% compare
figure(1);
subplot(2,1,1)
hold on;

% patch([0 sc_idx sc_idx 0], [-50 -50 50 50],brighten([0.5843 0.8157 0.9882],0.9));
% patch([sc_idx fl_idx fl_idx sc_idx], [-50 -50 50 50], brighten([0.8500 0.3250 0.0980],0.98));
% patch([fl_idx N N fl_idx], [-50 -50 50 50], brighten([0.4660 0.6740 0.1880],0.95));

x_imu = pos_imu(1:N, 4)-pos_imu(1,4); 
x_mocap = pos_mocap(:,1)-pos_mocap(1,1); % offset to the init point


h1=plot(x_imu,'-','LineWidth',2,'MarkerSize',10,'Color',[0.8500 0.3250 0.0980]);
h2=plot(x_mocap,'-','LineWidth',2,'MarkerSize',10,'Color',[0.9290 0.6940 0.1250]);

legend([h1, h2],{'x_{imu}','x_{mocap}'});
title("(a) x");
xlabel('t (ms)')
xlim([0 x_lim])
ylabel('x (m)')

subplot(2,1,2)
hold on;

% patch([0 sc_idx sc_idx 0], [-50 -50 50 50],brighten([0.5843 0.8157 0.9882],0.9));
% patch([sc_idx fl_idx fl_idx sc_idx], [-50 -50 50 50], brighten([0.8500 0.3250 0.0980],0.98));
% patch([fl_idx N N fl_idx], [-50 -50 50 50], brighten([0.4660 0.6740 0.1880],0.95));
z_imu = pos_imu(1:N, 6);
z_mocap = pos_mocap(:,3)-pos_mocap(1,3)+ h_init; % offset to the start point

h1=plot(z_imu,'-','LineWidth',2,'MarkerSize',10,'Color',[0.8500 0.3250 0.0980]);
h2=plot(z_mocap ,'-','LineWidth',2,'MarkerSize',10,'Color',[0.9290 0.6940 0.1250]);
% h3=plot(torque_thigh_RR_case3,'-','LineWidth',2,'MarkerSize',10,'Color',[0.4660 0.6740 0.1880]);

legend([h1, h2],{'z_{imu}','z_{mocap}'});
title("(b) z");
xlabel('t (ms)')
% xlim([0 z_lim])
ylabel('z (m)')

%% Traj from MoCap
figure(2);

pos_mocap_full = optitrack(1:N_full,1:3); % in m, order: x, y, z
h = plot(pos_mocap_full(:,1)-pos_mocap_full(1,1), pos_mocap_full(:,3)-pos_mocap_full(1,3), '-','LineWidth',2,'MarkerSize',10,'Color',[0.9290 0.6940 0.1250]);
legend([h],{'MoCap'});
% % legend([h1],{'x_{imu}'});
title("(a) CoM trajectory");
xlabel('x (m)')
ylabel('z (m)')
% 
% %% Traj from MoCap and IMU
% figure(3);
% hold on;
% 
% pos_mocap = M(start_jump:start_jump+N,7:9)/1000; % in m, order: x, y, z
% h1 = plot(pos_imu(1:N,4)-pos_imu(1,4), pos_imu(1:N,6),'-','LineWidth',2,'MarkerSize',10,'Color',[0.8500 0.3250 0.0980]);
% h2 = plot(pos_mocap(:,1)-pos_mocap(1,1), (pos_mocap(:,3)-pos_mocap(1,3)) + h_init, '-','LineWidth',2,'MarkerSize',10,'Color',[0.9290 0.6940 0.1250]);
% legend([h1, h2],{'imu','mocap'});
% % % legend([h1],{'x_{imu}'});
% title("CoM trajectory");
% xlabel('x (m)')
% ylabel('z (m)')
% 
%% Pitch angle from MoCap
figure(4)
quat_mocap_full = optitrack(1:N_full,4:7); % quaternion
orn_full = zeros(size(quat_mocap_full,1), 3); % roll pitch yaw
for i=1:size(quat_mocap_full,1)
    quat = [quat_mocap_full(i,1) quat_mocap_full(i,2) quat_mocap_full(i,3) quat_mocap_full(i,4)];
    orn_full(i,:) = quat2eul(quat, 'ZYX');
end

h = plot(orn_full(:,2)*180/3.14, '-','LineWidth',2,'MarkerSize',10,'Color',[0.9290 0.6940 0.1250]);
legend([h],{'Pitch'});
title("Pitch");
xlabel('t (ms)');
ylabel('pitch (degree)');

% %% Exporting to a new dataset (traj from MoCap + foot position from robot + force from robot)
% 
% % get orientation
% orn_mocap = zeros(size(quat_mocap,1), 3);
% for i=1:size(quat_mocap,1)
%     quat = [quat_mocap(i,4) quat_mocap(i,1) quat_mocap(i,2) quat_mocap(i,3)];
%     orn_mocap(i,:) = quat2eul(quat, 'ZYX');
% end
% 
% data_fcdc = dlmread('data_fcdc.txt');
% save('data_fcdc.mat','data_fcdc');
% data_fcdc(:,1:3) = [x_mocap/1000,z_mocap/1000,orn_mocap(:,2)]; % metter, metter, radian
% 
% % get velocity based on the position
% data_fcdc(1,4:6) = zeros(1,3); % velocity at start is 0
% % for i = 2:N
% %     data_fcdc(i,4) = (data_fcdc(i,1)-data_fcdc(i-1,1))/0.001; % vx
% %     data_fcdc(i,5) = (data_fcdc(i,2)-data_fcdc(i-1,2))/0.001; % vz
% %     data_fcdc(i,6) = (data_fcdc(i,3)-data_fcdc(i-1,3))/0.001; % wy
% % end


