function act = SOSflatOPT_SCALE(q1,q2,delta,Fext,r_par)

T=delta;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Parameters:
Scale_Link_1 = r_par(3)/0.33;
Scale_Link_2 = r_par(5)/0.32;
a0 = [0.1840*Scale_Link_1;0.1055*Scale_Link_1;...
      0.4283*Scale_Link_1;0.1916*Scale_Link_1;...
      0.2387*Scale_Link_2;0.1681*Scale_Link_2];
Ls = [0.0538*Scale_Link_1;0.0538*Scale_Link_1;...
      0.2298*Scale_Link_1;0.1905*Scale_Link_1;...
      0.1905*Scale_Link_2;0.0175*Scale_Link_2];
Lo = [0.1280*Scale_Link_1 0.1280*Scale_Link_1...
      0.1422*Scale_Link_1 0.0877*Scale_Link_1...
      0.0877*Scale_Link_2 0.1028*Scale_Link_2]';
Vm = 10*Lo;                                       
W  = 0.56*ones(6,1);                              
Fmax = [800;800;1000;1000;700;700];               
k = Fmax./(0.04*Ls).^2;                           
robotpars.m1 = r_par(1);
robotpars.m2 = r_par(2);
robotpars.l1 = r_par(3);
robotpars.lc1 = r_par(4);
robotpars.l2 = r_par(5);
robotpars.lc2 = r_par(6);
robotpars.I1 = r_par(7);
robotpars.I2 = r_par(8);
d1 = [0.05;-0.05;0.03;-0.03;0;0];
d2 = [0;0;0.03;-0.03;-0.03;0.03];
Di = [14.3   -7.1    1.4    0.4    0.4   -1.3;...
     -14.3    7.1    0.6   -0.4   -0.4    1.3;...
      -7.1   11.9   -0.7    0.3   -0.7    2.1;...
       7.1  -11.9    0.7    1.7    0.7   -2.1;...
      -7.1   -4.8   -0.7   -0.7    0.3    2.1;...
       7.1    4.8    0.7    0.7    1.7   -2.1];
Dy = Di(:,3:6);
Df = Di(:,1:2);
FTreserve = 20*ones(6,1);
g=9.81*0;
externalTorque = JvEval([q1(1);q2(1)],robotpars.l1,robotpars.l2)'*Fext';
[tau1,tau2,DfTau] = calcTauBounds2linkEnum(robotpars,q1,q2,g,externalTorque,Df);

% linear program: minimize f'x subject to Ax<= b and lb<=x
f = [T*ones(4,1);1/30*T^5*ones(4,1)];
A = [-eye(4) -1/16*T^4*eye(4); zeros(6,4) -Dy; -Dy -1/16*T^4*Dy; -Dy zeros(6,4)];
lb = zeros(8,1);
options = optimset('Display','none'); % silence output

% loop to automatically increase FTreserve, if needed
while FTreserve < min(Fmax) 

    B = (FTreserve-DfTau);
    b = [zeros(10,1);-B;-B];
    [x,~,exitflag] = linprog(f,A,b,[],[],lb,[],options);
        
    % identify any error
    if exitflag <= 0
        error('Solver failed:');
    end
        
    % evaluate resulting polynomial
    t=delta;        
    y3 = x(1)+x(5)*T^2*t.^2-2*x(5)*T*t.^3+x(5)*t.^4';
    y4 = x(2)+x(6)*T^2*t.^2-2*x(6)*T*t.^3+x(6)*t.^4';
    y5 = x(3)+x(7)*T^2*t.^2-2*x(7)*T*t.^3+x(7)*t.^4';
    y6 = x(4)+x(8)*T^2*t.^2-2*x(8)*T*t.^3+x(8)*t.^4';        
    ydot = [2*x(5)*T^2*t-6*x(5)*T*t.^2+4*x(5)*t.^3;
        2*x(6)*T^2*t-6*x(6)*T*t.^2+4*x(6)*t.^3;
        2*x(7)*T^2*t-6*x(7)*T*t.^2+4*x(7)*t.^3;
        2*x(8)*T^2*t-6*x(8)*T*t.^2+4*x(8)*t.^3;]';
        
% Calculate f1dot and f2dot by numerical differentiation
% (repeat last entry to maintain length
f1dot = diff([tau1 tau1(end)])/delta;
f2dot = diff([tau2 tau2(end)])/delta;

% calculate muscle state trajectories

% tendon forces
Phi = Di*[tau1;tau2;y3;y4;y5;y6];
    
% series element lengths
LS = SEEinv(Phi,Ls,k);
  
% LS rates 
denom = SEEPrime(LS,Ls,k);
num = Di*[f1dot;f2dot;ydot'];
LSdot = num./denom;
    
% LC, u and a
L = a0-d1*q1(1)-d2*q2(1);
Ldot = -d1*q1(2)-d2*q2(2);
LC = L-LS;
u = LSdot-Ldot;
FP = PE(LC,Lo);
act = (Phi-FP)./(f1(LC,W,Lo).*g1(u,Vm).*Fmax);
    
% if negative tendon force detected, increase FTreserve
if sum(sum(Phi<=0)) ~= 0
    worstCase = floor(min(min(Phi)));
    FTreserve = FTreserve + abs(worstCase)';
    continue
end

break
end

if sum(FTreserve >= min(Fmax))~=0
    error('Solver failed:');
end

end