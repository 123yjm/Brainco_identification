function get_excite_traj()
%% MATLAB激励轨迹优化, 匹配C++ 7-body模型
addpath('/home/ubuntu/Desktop/brainco_identification/matlab');
global h T wf N DOF q_init q_min q_max qd_lim qdd_lim

h=0.1; T=10; wf=2*pi/T; N=5; DOF=7;

q_init=[-1.5 0.15 0 0 -1.725 0 0]';
q_min=[-2.5 -0.15 -2.09 -0.3 -4.0 -0.54 -1.57]';
q_max=[0.7 1.65 2.09 2.21 0.55 0.54 1.57]';
qd_lim=deg2rad([80 80 80 80 80 80 80]');
qdd_lim=deg2rad([240 240 240 240 240 240 240]');

fprintf('=== MATLAB 激励轨迹优化 (C++模型) ===\n');
fprintf('采样: %d点, 变量: %d\n', T/h+1, DOF*(2*N+1));
tic;
rng(42); x0=rand(DOF*(2*N+1),1)*0.1;

options=optimoptions('fmincon','Display','iter','MaxIter',15000,'MaxFunEvals',1000000,...
    'Algorithm','sqp','OptimalityTolerance',1e-10,'StepTolerance',1e-10);

[x_opt,fval,exitflag]=fmincon(@obj,x0,[],[],[],[],[],[],@con,options);

fprintf('\n完成 (%.1f min), cond=%.4e\n', toc/60, fval);

% 解码
a=[x_opt(1:7) x_opt(8:14) x_opt(15:21) x_opt(22:28) x_opt(29:35)];
b=[x_opt(36:42) x_opt(43:49) x_opt(50:56) x_opt(57:63) x_opt(64:70)];
c=x_opt(71:77);

% 稠密轨迹 100Hz
fout=100; dt_out=1/fout; K_out=T*fout+1; time=(0:K_out-1)'*dt_out;
q_out=zeros(K_out,7); qd_out=zeros(K_out,7); qdd_out=zeros(K_out,7);
for k=1:K_out
    t=time(k); qk=c; qdk=zeros(7,1); qddk=zeros(7,1);
    for l=1:N
        qk=qk+a(:,l)*sin(l*wf*t)/(wf*l)-b(:,l)*cos(l*wf*t)/(wf*l);
        qdk=qdk+a(:,l)*cos(l*wf*t)+b(:,l)*sin(l*wf*t);
        qddk=qddk-a(:,l)*l*wf*sin(l*wf*t)+b(:,l)*l*wf*cos(l*wf*t);
    end
    q_out(k,:)=qk'; qd_out(k,:)=qdk'; qdd_out(k,:)=qddk';
end

% cond
Phi=[];
for k=0:h:T
    qk=c; qdk=zeros(7,1); qddk=zeros(7,1);
    for l=1:N
        qk=qk+a(:,l)*sin(l*wf*k)/(wf*l)-b(:,l)*cos(l*wf*k)/(wf*l);
        qdk=qdk+a(:,l)*cos(l*wf*k)+b(:,l)*sin(l*wf*k);
        qddk=qddk-a(:,l)*l*wf*sin(l*wf*k)+b(:,l)*l*wf*cos(l*wf*k);
    end
    Y=revoarm_right_getY_cpp(qk,qdk,qddk);
    Phi=[Phi;Y];
end
% 最终 cond(W_base)
scales2=sqrt(sum(Phi.^2,1)); scales2(scales2==0)=1;
[~,R2]=qr(Phi./scales2,0);
d2=abs(diag(R2)); tol2=100*eps*max(size(Phi))*d2(1); r2=sum(d2>tol2);
fprintf('最终 cond(W_base): %.4e (rank=%d)\n', d2(1)/d2(r2), r2);

% 保存
TBL=array2table([time q_out qd_out qdd_out zeros(K_out,7)],'VariableNames',...
    {'time','q0','q1','q2','q3','q4','q5','q6',...
    'qd0','qd1','qd2','qd3','qd4','qd5','qd6',...
    'qdd0','qdd1','qdd2','qdd3','qdd4','qdd5','qdd6',...
    'tau0','tau1','tau2','tau3','tau4','tau5','tau6'});
writetable(TBL,'../robots/revoarm_right/result_inertia/revoarm_right_excitation_trajectory.csv');
fprintf('CSV saved\n');
save('x_opt.mat','x_opt','a','b','c','fval');
end

function f=obj(x)
    global wf N
    a=[x(1:7) x(8:14) x(15:21) x(22:28) x(29:35)];
    b=[x(36:42) x(43:49) x(50:56) x(57:63) x(64:70)];
    c=x(71:77);
    Phi=[];
    for t=0:0.1:10
        qk=c; qdk=zeros(7,1); qddk=zeros(7,1);
        for l=1:5
            qk=qk+a(:,l)*sin(l*wf*t)/(wf*l)-b(:,l)*cos(l*wf*t)/(wf*l);
            qdk=qdk+a(:,l)*cos(l*wf*t)+b(:,l)*sin(l*wf*t);
            qddk=qddk-a(:,l)*l*wf*sin(l*wf*t)+b(:,l)*l*wf*cos(l*wf*t);
        end
        Y=revoarm_right_getY_cpp(qk,qdk,qddk);
        Phi=[Phi;Y];
    end
    % 列缩放 QRCP → cond(W_base)
    scales=sqrt(sum(Phi.^2,1)); scales(scales==0)=1;
    [~,R]=qr(Phi./scales,0);
    d=abs(diag(R)); eps_m=eps; tol=100*eps_m*max(size(Phi))*d(1);
    r=sum(d>tol);
    if r<2; f=1e10; else; f=d(1)/d(r); end
end

function [cneq,ceq]=con(x)
    global wf N q_init q_max q_min qd_lim qdd_lim
    a=[x(1:7) x(8:14) x(15:21) x(22:28) x(29:35)];
    b=[x(36:42) x(43:49) x(50:56) x(57:63) x(64:70)];
    c=x(71:77);
    q0=c; qd0=zeros(7,1); qdd0=zeros(7,1);
    for l=1:5
        q0=q0-b(:,l)/(wf*l); qd0=qd0+a(:,l); qdd0=qdd0+b(:,l)*wf*l;
    end
    ceq=[q0-q_init; qd0; qdd0];
    q_amp=abs(c-(q_max+q_min)/2); qd_amp=zeros(7,1); qdd_amp=zeros(7,1);
    for l=1:5
        amp=sqrt(a(:,l).^2+b(:,l).^2);
        q_amp=q_amp+amp/(wf*l); qd_amp=qd_amp+amp; qdd_amp=qdd_amp+amp*(wf*l);
    end
    cneq=[q_amp-(q_max-q_min)/2; qd_amp-qd_lim; qdd_amp-qdd_lim];
end
