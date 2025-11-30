function [C,b_opt,ds,H]=opt_poly(G)
% OPT_POLY Robust Pole Placement Controller Design
%
%   [C,b_opt,ds,H]=opt_poly(G) designs a robust controller C for the
%   given plant G using a polynomial approach based on spectral factorization
%   and pole placement.
%
%   Input G: The given system (TF, ZPK, or SS model).
%
%   Output C: The designed controller.
%   Output b_opt: Optimal gain parameter.
%   Output ds: Spectral factor polynomial.
%   Output H: Intermediate matrix in calculation.

[btemp,atemp] = tfdata(G);
a = atemp{1,1};
b = btemp{1,1};
 
na = length(a); 
as = tf(a,1);
bs = tf(b,1);

% Call Local Function: spectral
ds = spectral(a,b);
d = tfdata(ds,'v');
n = na-1;

J = zeros(n);
for i = 1:n
    J(i,i) = (-1)^(n-i);
end

sign = mod(na,2);
am = a;
for i = 1:2:na-1
    am(i+1) = -a(i+1);
end

if sign == 0 
    am = -am;
else 
    am = am;
end

bm = b;
for i = 1:2:na-1
    bm(i+1) = -b(i+1);
end

if sign == 0 
    bm = -bm;
else 
    bm = bm;
end

% Call Local Function: dioph
E = [dioph(bm,n),-dioph(am,n)]*[dioph(a,n),dioph(b,n)]^(-1)*dioph(d,n);
E0 = E(1:n,:);
F = dioph(d,n);
F0 = F(1:n,:);
H = J*F0^(-1)*E0;
[V,D] = eig(H);
DD = diag(D);

[gam,pos] = max(abs(DD));
e = V(:,pos);
e = e/e(1);
es = tf(e',1);
allp = roots(tfdata(es*ds,'v'));

% Call Local Function: poleplace (opt=1 for proper controller)
C = poleplace(G,allp,1);
b_opt=1/(1+gam^2)^.5;

end % End of main function: opt_poly

%% -------------------------------------------------------------
% Local Functions (Helper Functions)
%% -------------------------------------------------------------

function C=poleplace(P,p,opt)
% POLEPLACE  Pole placement
%   C=POLEPLACE(P,p,opt) returns a stabilizing controller C ...
%   (Original documentation omitted for brevity)

% Get the numerator and denominator of the system transfer function;
[btemp,atemp]=tfdata(P);
a=atemp{1,1};
b=btemp{1,1};
n=length(a)-1;

% Form the vector d which is the coefficents of the closed loop
% characteristic polynomial
dtemp=zpk(p,[],1);
dtemp1=tf(dtemp);
dtemp2=tfdata(dtemp1);
d=dtemp2{1,1};
nd=length(d);
m=nd-n-1;

% Form the matrix T(a,m+1)
Ta=zeros(n+m+1,m+1);
for i=1:m+1
   for j=i:i+n
       Ta(j,i)=a(j-i+1);
   end;
end;

if opt==1
    if m<n-1 error('The proper stabilizing controller does not exist'); end;
    % Form the matrix T(b,m+1)
    Tb=zeros(n+m+1,m+1);
    for i=1:m+1
        for j=i:i+n
            Tb(j,i)=b(j-i+1);
        end;
    end;
    T=[Ta Tb];
    q=T\d';

    for i=1:m+1
        den(i)=q(i);
        num(i)=q(m+1+i);
    end;
    C=tf(num,den);

else
    if m<n error('The strictly proper stabilizing controller does not exist'); end;

    % Form the matrix Taug(b,m)
    Tb=zeros(n+m+1,m);
    for i=1:m
        for j=i+1:i+n+1
            Tb(j,i)=b(j-i);
        end;
    end;
    T=[Ta Tb];
    q=T\d';

    % Fix: num array must be initialized correctly for the strictly proper case
    den = zeros(1, m+1);
    num = zeros(1, m+1);
    
    for i=1:m+1 den(i)=q(i);end;
    % The numerator is derived from q(m+2) to q(2*m+1) in the original code,
    % which implies the numerator degree is one less than the denominator.
    % We adjust the loop to correctly assign to the num vector indices.
    for i=1:m
        num(i+1)=q(m+1+i); % num[1] is 0 for strictly proper, coefficients start from num[2]
    end;
    C=tf(num,den);
end;
end

function ds=spectral(a,b)
% SPECTRAL  Spectral factorization
%   SPECTRAL(A,B) finds the spectral factorization of a given polynomial...
%   (Original documentation omitted for brevity)

na=length(a);
nb=length(b);

% Get a(-s) and b(-s)
signa=mod(na,2);signb=mod(nb,2);
am=a;bm=b;
for i=1:2:na-1
    am(i+1)=-a(i+1);
end;
if signa==0 am=-am;
else am=am;end;

for i=1:2:nb-1
    bm(i+1)=-b(i+1);
end;
if signb==0 bm=-bm;
else bm=bm;end;

% Find roots of a(-s)a(s)+b(-s)b(s)
as=tf(a,1);ams=tf(am,1);
bs=tf(b,1);bms=tf(bm,1);

cs=as*ams+bs*bms;
ctemp=tfdata(cs);
cc=ctemp{1,1};
c=cc/abs(cc(1));
d1=roots(c);

% Find spectral factor
nd1=length(d1);
d2=zeros(nd1/2,1);
j=1;
for i=1:nd1
    if real(d1(i))<0
        d2(j)=d1(i);
        j=j+1;
    end;
end;
d3=zpk(d2,[],1);
ds=sqrt(abs(cc(1)))*tf(d3);

end % End of local function: spectral

function [ Ta ] = dioph( a,m)
% DIOPH Generates the Sylvester matrix for Diophantine equations.
%   UNTITLED3 Summary of this function goes here
%   (Original documentation omitted for brevity)
n=m+1;
Ta=zeros(2*m,m);
for i=1:m
    for j=i:i+n-1
        Ta(j,i)=a(j-i+1);
    end;
end;
end % End of local function: dioph