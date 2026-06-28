addpath('/home/ubuntu/Desktop/brainco_identification/matlab');
global h T wf N; h=0.1; T=10; wf=2*pi/T; N=5;
rng(42); x0=rand(77,1)*0.1;
f=obj(x0);
fprintf('f at x0: %.6f (isnan=%d, isinf=%d)\n', f, isnan(f), isinf(f));

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
        try
            Y=revoarm_right_getY_cpp(qk,qdk,qddk);
        catch e
            fprintf('ERROR at t=%.2f: %s\n', t, e.message);
            f=NaN; return;
        end
        Phi=[Phi;Y];
    end
    f=cond(Phi);
end
