data = load(filename);

%sample = data(:,1);
%t = data(:,2);
% t = data(:,1)*0.01;
%t = (0:length(data(:,1))-1)*0.01;
t_abs = data(:,1);               % 绝对 Unix 时间戳 (秒)
t     = t_abs - t_abs(1);        % 相对时间，从 0 开始 (秒)
dof = 7;
offset = 1;
q              = data(:,offset+1:offset+dof)';   offset = offset + dof;
q_dot          = data(:,offset+1:offset+dof)';   offset = offset + dof;   
motor_current  = data(:,offset+1:offset+dof)';   offset = offset + dof;
qd             = data(:,offset+1:offset+dof)';   offset = offset + dof;
qd_dot         = data(:,offset+1:offset+dof)';   offset = offset + dof;
qd_ddot        = data(:,offset+1:offset+dof)';   offset = offset + dof;
%jointacttorque = data(:,offset+1:offset+dof)';   offset = offset + dof; %这个值不用

ratio = [1;1;1;1;1;1:1];
for i = 1:dof
    q_dot(i,:) = q_dot(i,:);
end

tau_cof=[1, 1, 1, 1, 1, 1, 1]; 
for i = 1:dof
    motor_current(i,:) = motor_current(i,:)*tau_cof(i);
end
clear data
