clear all; close all;

fs = 1e3; % 1KHz, 
f_filter = 33; % mpc Frequency ( 1/dt_mpc)


h_init = 0.1434; % initial robot height
data_f = dlmread('data_f.txt'); % no limits
save('data_f.mat','data_f');
pos = dlmread('pos.txt'); % no limits
save('pos.mat','pos');
N = size(data_f,1); % number of data point 



data_pd = dlmread('data_pd.txt'); % no limits
save('data_pd.mat','data_pd');
% data_fmdc = dlmread('data_fmdc.txt'); % no limits
% save('data_fmdc.mat','data_fmdc');
data_fcmd = dlmread('data_fcmd.txt'); % no limits
save('data_fcmd.mat','data_fcmd');

joint_act = dlmread('joint_act.txt'); % no limits
save('joint_act.mat','joint_act');

joint_des = dlmread('joint_des.txt'); % no limits
save('joint_des.mat','joint_des');

torque = dlmread('torque.txt'); % no limits
save('torque.mat','torque');

% % don't want to change joint des during landing preparation
% for i=1100:N
%     joint_act(i, 5) = joint_act(1100,5);
%     joint_act(i, 7) = joint_act(1100,7);
%     joint_act(i, 9) = joint_act(1100,9);
%     joint_act(i, 11) = joint_act(1100,11);
%     joint_des(i, 5) = joint_des(1100,5);
%     joint_des(i, 7) = joint_des(1100,7);
%     joint_des(i, 9) = joint_des(1100,9);
%     joint_des(i, 11) = joint_des(1100,11);
% end

q_front_thigh_act = joint_act(:,5); q_front_thigh_des = joint_des(:,5); 
q_front_calf_act = joint_act(:,9); q_front_calf_des = joint_des(:,9); 
dq_front_thigh_act = joint_act(:,17); dq_front_thigh_des = joint_des(:,17); 
dq_front_calf_act = joint_act(:,21); dq_front_calf_des = joint_des(:,21); 

q_rear_thigh_act = joint_act(:,7); q_rear_thigh_des = joint_des(:,7); 
q_rear_calf_act = joint_act(:,11); q_rear_calf_des = joint_des(:,11); 
dq_rear_thigh_act = joint_act(:,19); dq_rear_thigh_des = joint_des(:,19); 
dq_rear_calf_act = joint_act(:,23); dq_rear_calf_des = joint_des(:,23); 


Kp = 300; Kd=3;
tau_pd = zeros(N,4);
tau_pd(:,1) = 2*Kp*(q_front_thigh_des-q_front_thigh_act) + 2*Kd*(dq_front_thigh_des-dq_front_thigh_act);
tau_pd(:,2) = 2*Kp*(q_front_calf_des-q_front_calf_act) + 2*Kd*(dq_front_calf_des-dq_front_calf_act);
tau_pd(:,3) = 2*Kp*(q_rear_thigh_des-q_rear_thigh_act) + 2*Kd*(dq_rear_thigh_des-dq_rear_thigh_act);
tau_pd(:,4) = 2*Kp*(q_rear_calf_des-q_rear_calf_act) + 2*Kd*(dq_rear_calf_des-dq_rear_calf_act);

start_impact = 1030;
for i=start_impact+1:N
    tau_pd(i,1)=tau_pd(start_impact,1);
    tau_pd(i,2)=tau_pd(start_impact,2);
    tau_pd(i,3)=tau_pd(start_impact,3);
    tau_pd(i,4)=tau_pd(start_impact,4);
end

% Compute rotation matrix based on pitch angle
% N = 1000;

tau_pd_test = zeros(N,4);
f_pd = zeros(N,4);
for i=1:N
    
    % Note the sign is important ( match with Pybullet --checked 03/09/24)
    Ri = [cos(pos(i,8)), sin(pos(i,8)); 
          -sin(pos(i,8)), cos(pos(i,8))];
    % front leg
    q = [q_front_thigh_act(i), q_front_calf_act(i)];
    [J,p] = computeLegJacobianAndPosition_2D(q);
    temp1 = -inv(transpose(J)*transpose(Ri))*transpose(tau_pd(i,1:2)); % force x z
    f_pd(i,1:2) = transpose(temp1); % x an z force
    % % convert back to torque pd to validate
    % temp1 = -transpose(J)*transpose(Ri)*temp1; % torque hip, 
    % tau_pd_test(i,1:2) = transpose(temp1);

    % rear leg
    q = [q_rear_thigh_act(i), q_rear_calf_act(i)];
    [J,p] = computeLegJacobianAndPosition_2D(q);
    temp2 = -inv(transpose(J)*transpose(Ri))*transpose(tau_pd(i,3:4)); % force x z
    f_pd(i,3:4) = transpose(temp2); % x an z force

    % % % convert back to torque pd to validate
    % temp2 = -transpose(J)*transpose(Ri)*temp2;
    % tau_pd_test(i,3:4) = transpose(temp2);

end
f_pd_filter = zeros(N,4);
for i=1:4
    f_pd_filter(:,i) = lowpass(f_pd(:,i), f_filter, fs); 
end

f_cmd_new = zeros(N,4); % combine feedforward and PD
for i=1:N

    % Note the sign is important
    Ri = [cos(pos(i,8)), sin(pos(i,8)); 
          -sin(pos(i,8)), cos(pos(i,8))];
    % front leg
    q = [q_front_thigh_act(i), q_front_calf_act(i)];
    [J,p] = computeLegJacobianAndPosition_2D(q);
    temp1 = -inv(transpose(J)*transpose(Ri))*transpose(2*[torque(i,25),torque(i,29)]); % force x z
    f_cmd_new(i,1:2) = transpose(temp1); % x an z force
    % % convert back to torque pd to validate
    % temp1 = -transpose(J)*transpose(Ri)*temp1; % torque hip, 
    % tau_pd_test(i,1:2) = transpose(temp1);

    % rear leg
    q = [q_rear_thigh_act(i), q_rear_calf_act(i)];
    [J,p] = computeLegJacobianAndPosition_2D(q);
    temp2 = -inv(transpose(J)*transpose(Ri))*transpose(2*[torque(i,27),torque(i,31)]); % force x z
    f_cmd_new(i,3:4) = transpose(temp2); % x an z force

    % % % convert back to torque pd to validate
    % temp2 = -transpose(J)*transpose(Ri)*temp2;
    % tau_pd_test(i,3:4) = transpose(temp2);

end

f_cmd_total = f_cmd_new + f_pd_filter;

% tau_pd_test = zeros(N,4);
% f_cmd_total = zeros(N,4); % combine feedforward and PD
% for i=1:N
% 
%     % Note the sign is important
%     Ri = [cos(-pos(i,8)), -sin(-pos(i,8)); 
%           sin(-pos(i,8)), cos(-pos(i,8))];
%     % front leg
%     q = [q_front_thigh_act(i), q_front_calf_act(i)];
%     [J,p] = computeLegJacobianAndPosition_2D(q);
%     temp1 = -inv(transpose(J)*transpose(Ri))*transpose(2*[torque(i,25),torque(i,29)]+tau_pd(i,1:2)); % force x z
%     f_cmd_total(i,1:2) = transpose(temp1); % x an z force
%     % % convert back to torque pd to validate
%     % temp1 = -transpose(J)*transpose(Ri)*temp1; % torque hip, 
%     % tau_pd_test(i,1:2) = transpose(temp1);
% 
%     % rear leg
%     q = [q_rear_thigh_act(i), q_rear_calf_act(i)];
%     [J,p] = computeLegJacobianAndPosition_2D(q);
%     temp2 = -inv(transpose(J)*transpose(Ri))*transpose(2*[torque(i,27),torque(i,31)]+tau_pd(i,3:4)); % force x z
%     f_cmd_total(i,3:4) = transpose(temp2); % x an z force
% 
%     % % % convert back to torque pd to validate
%     % temp2 = -transpose(J)*transpose(Ri)*temp2;
%     % tau_pd_test(i,3:4) = transpose(temp2);
% 
% end

% f_pd_3d = zeros(N,6);
% f_pd = zeros(N,4);
% tau_pd_test = zeros(N,6);
% for i=1:N
% 
%     % Sign match with Pybullet ( 03/09/24 - checked?)
%     Ri = [cos(pos(i,8)), 0, sin(pos(i,8));
%           0, 1, 0;
%           -sin(pos(i,8)), 0,  cos(pos(i,8))];
%     % front leg
%     q = [0, q_front_thigh_act(i), q_front_calf_act(i)];
%     [J,p] = computeLegJacobianAndPosition(q, 1);
%     % [J,p] = computeLegJacobianAndPosition_2D(q);
%     temp1 = -inv(transpose(J)*transpose(Ri))*transpose([0,tau_pd(i,1:2)]); % force x z
%     f_pd_3d(i,1:3) = transpose(temp1); % 
%     % % convert back to torque pd to validate
%     % temp1 = -transpose(J)*transpose(Ri)*temp1; % torque hip, 
%     % tau_pd_test(i,1:3) = transpose(temp1);
% 
%     % rear leg
%     q = [0, q_rear_thigh_act(i), q_rear_calf_act(i)];
%     [J,p] = computeLegJacobianAndPosition(q,2);
%     temp2 = -inv(transpose(J)*transpose(Ri))*transpose([0,tau_pd(i,3:4)]); % force x z
%     f_pd_3d(i,4:6) = transpose(temp2); % 
% 
%     % % % convert back to torque pd to validate
%     % temp2 = -transpose(J)*transpose(Ri)*temp2;
%     % tau_pd_test(i,4:6) = transpose(temp2);
% 
% end
% f_pd(:,1) = f_pd_3d(:,1);
% f_pd(:,2) = f_pd_3d(:,3);
% f_pd(:,3) = f_pd_3d(:,4);
% f_pd(:,4) = f_pd_3d(:,6);


% chuong = tau_pd-tau_pd_test;

figure(1);
subplot(2,1,1)
hold on;


h1=plot(q_rear_thigh_act,'-','LineWidth',2,'MarkerSize',10,'Color',[0.8500 0.3250 0.0980]);
h2=plot(q_rear_thigh_des,'-','LineWidth',2,'MarkerSize',10,'Color',[0.9290 0.6940 0.1250]);

legend([h1, h2],{'q_{act}','q_{des}'});
title("(a) rear thigh");
xlabel('t (ms)')
ylabel('x (rad)')

subplot(2,1,2)
hold on;

h1=plot(q_rear_calf_act,'-','LineWidth',2,'MarkerSize',10,'Color',[0.8500 0.3250 0.0980]);
h2=plot(q_rear_calf_des,'-','LineWidth',2,'MarkerSize',10,'Color',[0.9290 0.6940 0.1250]);

legend([h1, h2],{'q_{act}','q_{des}'});
title("(a) rear calf");
xlabel('t (ms)')
ylabel('x (rad)')
%%
figure(2);
subplot(2,1,1)
hold on;
h1=plot(dq_rear_thigh_act,'-','LineWidth',2,'MarkerSize',10,'Color',[0.8500 0.3250 0.0980]);
h2=plot(dq_rear_thigh_des,'-','LineWidth',2,'MarkerSize',10,'Color',[0.9290 0.6940 0.1250]);

legend([h1, h2],{'qd_{act}','qd_{des}'});
title("(a) rear thigh");
xlabel('t (ms)')
ylabel('x (rad/s)')

subplot(2,1,2)
hold on;

h1=plot(dq_rear_calf_act,'-','LineWidth',2,'MarkerSize',10,'Color',[0.8500 0.3250 0.0980]);
h2=plot(dq_rear_calf_des,'-','LineWidth',2,'MarkerSize',10,'Color',[0.9290 0.6940 0.1250]);

legend([h1, h2],{'qd_{act}','qd_{des}'});
title("(a) rear calf");
xlabel('t (ms)')
ylabel('x (rad/s)')


%%
figure(3);
subplot(2,1,1)
hold on;

Fx = data_f(:,12); 
Fz = data_f(:,13); 


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'F_{x}','F_{z}'});
title("(a) front leg");
xlabel('t (ms)')
ylabel('F (N)')

subplot(2,1,2)
hold on;

Fx = data_f(:,14); 
Fz = data_f(:,15); 


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'F_{x}','F_{z}'});
title("(a) rear leg");
xlabel('t (ms)')
ylabel('F (rad)')

%%
figure(4);
subplot(2,1,1)
hold on;

Fx = f_pd(:,1); 
Fz = f_pd(:,2); 


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'F_{x}','F_{z}'});
title("(a) f_{pd} front leg");
xlabel('t (ms)')
ylabel('F (N)')

subplot(2,1,2)
hold on;

Fx = f_pd(:,3); 
Fz = f_pd(:,4); 


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'F_{x}','F_{z}'});
title("(a) f_{pd} rear leg");
xlabel('t (ms)')
ylabel('F (N)')

%% f_pd_filter
figure(5)
subplot(2,1,1)
hold on;

Fx = f_pd_filter(:,1); 
Fz = f_pd_filter(:,2); 


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'F_{x}','F_{z}'});
title("(a) f_{pd} filter front leg");
xlabel('t (ms)')
ylabel('F (N)')

subplot(2,1,2)
hold on;

Fx = f_pd_filter(:,3); 
Fz = f_pd_filter(:,4); 


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'F_{x}','F_{z}'});
title("(a) f_{pd} filtered rear leg");
xlabel('t (ms)')
ylabel('F (N)')
%% compare f_pd and f_pd filter
figure(6)
subplot(2,1,1)
hold on;

f1 = f_pd(:,2); 
f2 = f_pd_filter(:,2); 


h1=plot(f1,'-','LineWidth',1,'MarkerSize',10,'Color','k');
h2=plot(f2,'-','LineWidth',2,'MarkerSize',10,'Color','r');

legend([h1, h2],{'F_{z}','F_{z} filter'});
title("(a) compare f_{pd} and f_{pd} filter front leg");
xlabel('t (ms)')
ylabel('F (N)')

subplot(2,1,2)
hold on;

f1 = f_pd(:,4); 
f2 = f_pd_filter(:,4); 


h1=plot(f1,'-','LineWidth',1,'MarkerSize',10,'Color','k');
h2=plot(f2,'-','LineWidth',2,'MarkerSize',10,'Color','r');

legend([h1, h2],{'F_{z}','F_{z} filter'});
title("(b) compare f_{pd} and f_{pd} filter rear leg");
xlabel('t (ms)')
ylabel('F (N)')
%% Fmpc+ F_pd filter
figure(7);
subplot(2,1,1)
hold on;

Fx = f_pd_filter(:,1) + data_f(:,12); 
Fz = f_pd_filter(:,2) + data_f(:,13); 


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'F_{x}','F_{z}'});
title("(a) f_{mpc}+ f_{pd} filter front leg");
xlabel('t (ms)')
ylabel('F (N)')

subplot(2,1,2)
hold on;

Fx = f_pd_filter(:,3) + data_f(:,14); 
Fz = f_pd_filter(:,4) + data_f(:,15); 


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'F_{x}','F_{z}'});
title("(a) f_{mpc} + f_{pd} filter rear leg");
xlabel('t (ms)');
ylabel('F (N)');
%% fcmd_new
figure(8);
subplot(2,1,1)
hold on;

Fx = f_cmd_new(:,1); 
Fz = f_cmd_new(:,2); 


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'Fcmd_{x}','Fcmd_{z}'});
title("(a) front leg");
xlabel('t (ms)')
ylabel('F (N)')

subplot(2,1,2)
hold on;

Fx = f_cmd_new(:,3); 
Fz = f_cmd_new(:,4); 


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'Fcmd_{x}','Fcmd_{z}'});
title("(a) rear leg");
xlabel('t (ms)')
ylabel('F (N)')

%% fcmd_total
figure(9);
subplot(2,1,1)
hold on;

Fx = f_cmd_total(:,1); 
Fz = f_cmd_total(:,2) ;


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'Fx_{total}','Fz_{total}'});
title("(a) front leg");
xlabel('t (ms)')
ylabel('F (N)')

subplot(2,1,2)
hold on;

Fx = f_cmd_total(:,3); 
Fz = f_cmd_total(:,4) ;


h1=plot(Fx,'-','LineWidth',2,'MarkerSize',10,'Color','k');
h2=plot(Fz,'-','LineWidth',2,'MarkerSize',10,'Color','b');

legend([h1, h2],{'Fx_{total}','Fz_{total}'});
title("(a) rear leg");
xlabel('t (ms)')
ylabel('F (N)')

%% Export data

data_fmpc_pd = data_fcmd;
data_fmpc_pd(:,12:13) = f_pd_filter(:,1:2) + data_f(:, 12:13);
data_fmpc_pd(:,14:15) = f_pd_filter(:,3:4) + data_f(:, 14:15);

data_fcmd_new = data_fcmd;
data_fcmd_new(:,12:15) = f_cmd_new;

data_fcmd_total = data_fcmd;
data_fcmd_total(:,12:15) = f_cmd_total;

% %% export back to txt

writematrix(data_fmpc_pd,'data_fmpc_pd.txt');
writematrix(data_fcmd_new,'data_fcmd_new.txt');
writematrix(data_fcmd_total,'data_fcmd_total.txt');

