%dvgpro%

function [Sf,Sp,K]=feedback_matrix(s,P,M,N,w)

Sf=zeros(P,M);

for i=1:M
    Sf(:,i)=[zeros(i-1,1);s(1:P-i+1)];
end

Sp=zeros(P,N-2);

for i=1:P
    Sp(i,:)=[s(i+1:N-1)' zeros(1,i-1)];
end

K=inv(Sf'*Sf+w*eye(M))*Sf';

end

