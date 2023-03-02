%dvgpro%

function delu=optimal_controlmove(Sp,K,sn,delup,d,r,u,k,N)
% determining optimal control move

[mm,pp]=size(K);

uold=zeros(pp,1);
for i=1:pp
    if k-N+i>0
        uold(i,1)=u(k-N+i);
    else
        uold(i,1)=0;
    end
end
dvec = d*ones(pp,1);
rvec = r*ones(pp,1);   % setpoint for P timesteps

% model prediction over prediction horizon of P
y_free = Sp*delup+sn*uold+dvec; 
% prediction error
e_free = rvec-y_free;
% optimal control move
delu = K(1,:)*e_free;

end
