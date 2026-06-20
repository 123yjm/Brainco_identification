
DOF = 7;

%%  generate butterworth filter paras a b vector
Ts = t(2)-t(1);
fs = 1/Ts;
% Wp = 40/(fs/2);     %Stopband corner frequency Ws, 1 corresponding to the normalized Nyquist frequency
% Ws = 160/(fs/2);    %Passband corner frequency Wp, normalized via dividing by nyquist frequency fs/2;
% Wp = 10/(fs/2);     %Stopband corner frequency Ws, 1 corresponding to the normalized Nyquist frequency
% Ws = 60/(fs/2);    %Passband corner frequency Wp, normalized via dividing by nyquist frequency fs/2;
Wp = 5/(fs/2);     %Stopband corner frequency Ws, 1 corresponding to the normalized Nyquist frequency
Ws = 10/(fs/2);    %Passband corner frequency Wp, normalized via dividing by nyquist frequency fs/2;
Rp = 3;             %Passband ripple in decibels
Rs = 30;            %Stopband attenuation in decibels. This value is the number of decibels the stopband is down from the passband.
[n,Wn] = buttord(Wp,Ws,Rp,Rs);
[b,a] = butter(n,Wn);       %cutoff frequencies Wn and the lowest order n of the digital Butterworth filter 

%% Plot The Butterworth Filter
% figure(10)
% [z,p,k] = butter(n,Wn);
% sos = zp2sos(z,p,k);
% freqz(sos,512,fs)
% title(sprintf('n = %d Butterworth Lowpass Filter',n))

%% filt jointTorque and jointPos
motor_current_filtered = filtfilt(b,a,motor_current(:,:));

q_filtered = q;


%% Central difference to get jointVel and jointAcc
% jointVel
q_dot_filtered = filtfilt(b,a,q_dot(:,:));


% jointAcc
q_ddot_filtered = zeros(size(q_dot_filtered));
for i = 2:size(q_dot_filtered,1)-1
    q_ddot_filtered_i = (q_dot_filtered(i+1,:) - q_dot_filtered(i-1,:))/(2*Ts); % central difference
    q_ddot_filtered(i,:) = q_ddot_filtered_i;
end
q_ddot_filtered(i,1) = q_ddot_filtered(i,2);
q_ddot_filtered(i,end) = q_ddot_filtered(i,end-1);


%% plot
fig_cnt = 1;

% sensor torque
% figure(fig_cnt)
% fig_cnt = fig_cnt + 1;
% for i = 1:DOF
%     subplot(3,3,i)
%     plot(t,jointacttorque(:,i))
%     hold on
%     plot(t,jointacttorque_filtered(:,i))
%     %plot(t,jointacttorque_filtered(:,i)-jointacttorque(:,i))
%     xlabel('time/s');ylabel('torque/N*m');title('Lowpass Butterworth Filter Torque')
%     axis tight
%     if i == DOF
%         legend('jointacttorque','jointacttorque\_filtered')
%     end
% end

% filtered current
figure(fig_cnt)
fig_cnt = fig_cnt + 1;
for i = 1:DOF
    subplot(3,3,i)
    plot(t,motor_current(:,i))
    hold on
    plot(t,motor_current_filtered(:,i))
    xlabel('time/s');ylabel('motor current/A');title('Lowpass Butterworth Filter Current')
    axis tight
    if i == DOF
        legend('motor\_current','motor\_current\_filtered')
    end
end

% actual position and filtered actual position
figure(fig_cnt)
fig_cnt = fig_cnt + 1;
for i = 1:DOF
    subplot(3,3,i)
    plot(t,q(:,i))
    hold on
    plot(t,q_filtered(:,i))
    hold on 
    plot(t,qd(:,i))
    %plot(t,q(:,i)-q_filtered(:,i))
    xlabel('time/s');ylabel('jointpos/rad');title('Lowpass Butterworth Filter JointPos')
    axis tight
end

% filtered actual velocity
figure(fig_cnt)
fig_cnt = fig_cnt + 1;
for i = 1:DOF
    subplot(3,3,i)
    plot(t,q_dot_filtered(:,i))
    hold on 
    plot(t,qd_dot(:,i))
    xlabel('time/s');ylabel('jointvel/rad/s');title('Lowpass Butterworth Filter JointVel')
    axis tight
    if i == DOF
        legend('q\_dot\_filtered','qd\_dot')
    end
end

% filtered actual acceleration
figure(fig_cnt)
fig_cnt = fig_cnt + 1;
for i = 1:DOF
    subplot(3,3,i)
    plot(t,q_ddot_filtered(:,i))
    hold on 
    plot(t,qd_ddot(:,i))
    xlabel('time/s');ylabel('jointacc/rad/s^2');title('Lowpass Butterworth Filter JointAcc')
    axis tight
    if i == DOF
        legend('q\_ddot\_filtered','qd\_ddot')
    end
end



%% save data
% save('filter_paras.mat','a','b','Ts')
% save('filtered_data.mat','jointacttorque_filtered','q_filtered','q_dot_filtered','q_ddot_filtered','t')


