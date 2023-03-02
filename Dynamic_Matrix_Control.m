%dvgpro%

% Unconstrained Dynamic marix control
% B. W. Bequette

clc
clear all

% MPC parameters

N=50;  % model horizon
P=20;  % prediction horizon
M=1;   % control horizon
w=0;
ysp=1; % setpoint chnage
timesp=1; % time of setpoint change
delt=0.1; % sampling interval
tfinal=6;
noise=0;

t=0:delt:tfinal;
kfinal=length(t);

dd=timesp/delt;
r=[zeros(1,dd) ones(1,kfinal-dd)];

% process model
a=[-2.4048 0;0.8333 -2.2381];
b=[7;-1.117];
c=[0 1];
d=0;
Sysc_model=ss(a,b,c,d);

% actual plant

Sysc_plant=Sysc_model;
Sysd_plant=c2d(Sysc_plant,delt);
[A,B,C,D]=ssdata(Sysd_plant);

% step response coefficients
s=step(Sysc_model,[delt:delt:N*delt]);

[Sf,Sp,K]=feedback_matrix(s,P,M,N,w);

% plant initial conditions
xinit=zeros(length(a),1);
uinit=0;
yinit=0;

% initialize input vector
u=zeros(min(P,kfinal),1)*uinit;
dup=zeros(N-2,1);
sn=s(N); 
x(:,1)=xinit;
y(1)=yinit;
dist(1)=0;

for k=1:kfinal
    % current optimal control move
    du(k)=optimal_controlmove(Sp,K,sn,dup,dist(k),r(k),u,k,N);
    if k>1
        u(k)=u(k-1)+du(k);
    else
        u(k)=uinit+du(k);
    end
    % actual process output for u(k)
    x(:,k+1)=A*x(:,k)+B*u(k);
    y(k+1)=C*x(:,k+1);
    % model prediction for u(k)
    if k-N+1>0
        ymod(k+1)=s(1)*du(k)+Sp(1,:)*dup+sn*u(k-N+1);
    else
        ymod(k+1)=s(1)*du(k)+Sp(1,:)*dup;
    end
    % difference between the actual process output and the model prediction
    dist(k+1)=y(k+1)-ymod(k+1);
    % store the current control move into the vector of past control moves
    dup=[du(k);dup(1:N-3)];
end

[tt,uu]=stairs(t,u);
[ttr,rr]=stairs(t,r);

figure
subplot(2,1,1)
plot(ttr,rr,'--',t,y(1:kfinal))
ylabel('y')
xlabel('time')
subplot(2,1,2)
plot(tt,uu)
xlabel('time')
ylabel('u')







