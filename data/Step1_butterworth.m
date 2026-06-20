
%% init
clear;clc;close all
%cond_num = '57.93';
%cond_num = '95.6';
%cond_num = '103.8';
cond_num = '56.12';
% filename = '/home/ubuntu/Desktop/system_identification/revoarm_dynamic/para_est_CP/revoarm_bimanual/revoarm_right_data_read_excitation_56.12_right.txt';
% filename = '/home/ubuntu/Desktop/system_identification/revoarm_dynamic/para_est_CP/revoarm_bimanual/revoarm_right_data_read_excitation_56.12_adjusted_20260617_232559.txt';
% filename = '/home/ubuntu/Desktop/system_identification/revoarm_dynamic/para_est_CP/revoarm_bimanual/revoarm_right_data_read_excitation_56.12_right.txt';
% filename = '/home/ubuntu/Desktop/system_identification/revoarm_dynamic/para_est_CP/revoarm_bimanual/revoarm_right_data_read_excitation_56.12_0618.txt';
% filename = '/home/ubuntu/Desktop/Robot-Parameter-Indentification-Simulation/data/revoarm_right_data_read_excitation_56.12_0616.txt';
filename = '/home/ubuntu/Desktop/Robot-Parameter-Indentification-Simulation/data/revoarm_right_data_read_excitation_56.12_adjusted_20260618_005737.txt';
txt2data_revoarm;

%%
motor_current=motor_current';
q=q';
q_dot=q_dot';
qd = qd';
qd_dot=qd_dot';
qd_ddot=qd_ddot';

revoarm_filter;


%% save data
%save(['filter_paras.mat'],'a','b','Ts')
% save(['filtered_data_x_',cond_num,'.mat'],'jointacttorque_filtered','q_filtered','q_dot_filtered','q_ddot_filtered')
jointacttorque_filtered = motor_current_filtered;
save(['revoarm_filtered_data_condnum_',cond_num,'.mat'],'jointacttorque_filtered','q_filtered','q_dot_filtered','q_ddot_filtered')



