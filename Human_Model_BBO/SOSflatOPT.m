% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% Based on code for paper "Motion Optimization for Musculoskeletal
% Dynamics: A Flatness-Based Sum-of-Squares Approach"
% by Hanz Richter and Holly Warner
% Cleveland State University, Center for Human-Machine Systems, 2019
% 
% File description: Flatness-based SOS open-loop optimization solver

function [t,n,act,LS,u,Phi,q1,q2,FTreserve] = SOSflatOPT(q1,q2,robotpars,musclepars,g,T,delta,tau_act,beta_act,externalTorque,useSOSTools)

% parse muscle parameters
d1 = musclepars.d1;
d2 = musclepars.d2;
a0 = musclepars.a0;
Ls = musclepars.Ls;
Lo = musclepars.Lo;
Vm = musclepars.Vm;
W = musclepars.W;
Fmax = musclepars.Fmax;
k = musclepars.k;

ArmMatrix = [d1';d2'];

% distribution matrix
D = [ArmMatrix;0.5 0.5 0 0 0 0;0 0 0.5 0.5 0 0;0 0 0 0 0.5 0.5;1/3 0 1/3 0 1/3 0];
 
% distribution matrix inverse
Di = inv(D);

% divide Di
Dy = Di(:,3:6);
Df = Di(:,1:2);

% find the torque bounds for the above trajectories
% tic    
[tau1,tau2,DfTau] = calcTauBounds2linkEnum(T,robotpars,q1,q2,delta,g,0,externalTorque,Df);
% toc

% assign tendon force reserve
FTreserve = 20*ones(6,1);

% polynomial optimization for y3,y4,y5 and y6 using SOSTools
if useSOSTools
    
    % define variable t
    syms t
    
    % initialize SOS program
    Program1 = sosprogram(t);
    
    % fourth order polynomial
    vec2 = monomials(t,[0 1 2 3 4]);
    
    % define polynomial matrix (4 co-contraction polynomials being solved)
    [Program1,v2] = sospolymatrixvar(Program1,vec2,[4 1]);

    % assign integral co-contraction cost
    area = sum(int(v2,[0 T]));
    Program1 = sossetobj(Program1,area);
    
    % periodicity constraints
    Program1 = soseq(Program1,subs(v2,t,0)-subs(v2,t,T));
    v2dot = diff(v2);
    v2ddot = diff(v2dot);
    Program1 = soseq(Program1,subs(v2dot,t,0)-subs(v2dot,t,T));
    Program1 = soseq(Program1,subs(v2ddot,t,0)-subs(v2ddot,t,T));

    % left hand side of inequality constraints for y3, y4, y5, and y6    
    leftside = Dy*v2;
    
% otherwise, use linear programming    
else

    % linear program: minimize f'x subject to Ax<= b and lb<=x
    f = [T*ones(4,1);1/30*T^5*ones(4,1)];
    A = [-eye(4) -1/16*T^4*eye(4); zeros(6,4) -Dy; -Dy -1/16*T^4*Dy; -Dy zeros(6,4)];
    lb = zeros(8,1);
    options = optimset('Display','none'); % silence output

end

% loop to automatically increase FTreserve, if needed
while FTreserve < min(Fmax) 

    % polynomial optimization for y3,y4,y5 and y6 using SOSTools
    if useSOSTools
                
        % save SOS program in temporary variable to avoid having to
        % recreate it with each iteration updating FTreserve
        Program1Temp = Program1;
        
        act = [];
        
        % right hand side of inequality constraints for y3, y4, y5, and y6    
        rightside = FTreserve-DfTau;
        
        % create inequality
        Program1Temp = sosineq(Program1Temp,leftside-rightside,[0 T]);
        
        % solve the SOS problem and extact solution
        evalc('Program1Temp = sossolve(Program1Temp)');        
        vsol2 = sosgetsol(Program1Temp,v2);
        
        % identify any solution errors
        if Program1Temp.solinfo.info.pinf ~= 0
            errorMsg = 'Primal problem infeasible';
        elseif Program1Temp.solinfo.info.dinf ~= 0
            errorMsg = 'Dual problem infeasible';
        elseif Program1Temp.solinfo.info.numerr ~= 0
            errorMsg = 'Numerical issues';
        end
        
        if exist('errorMsg')
            error(['Solver failed: ' errorMsg ' with feasibility ratio ' num2str(Program1Temp.solinfo.info.feasratio)])
        end
        
        % evaluate resulting polynomials
        t = [0:delta:T];        
        y3 = eval(vsol2(1));
        y4 = eval(vsol2(2));
        y5 = eval(vsol2(3));
        y6 = eval(vsol2(4));
        ydot = eval(diff(vsol2))';
        
    % otherwise, use linear programming    
    else
        
        B = (FTreserve-DfTau);
        b = [zeros(10,1);-B;-B];
        [x,~,exitflag] = linprog(f,A,b,[],[],lb,[],options);
        
        % identify any error
        if exitflag <= 0
            error('Solver failed:');
        end
        
        % evaluate resulting polynomial
        t = [0:delta:T];        
        y3 = [x(1)+x(5)*T^2*t.^2-2*x(5)*T*t.^3+x(5)*t.^4]';
        y4 = [x(2)+x(6)*T^2*t.^2-2*x(6)*T*t.^3+x(6)*t.^4]';
        y5 = [x(3)+x(7)*T^2*t.^2-2*x(7)*T*t.^3+x(7)*t.^4]';
        y6 = [x(4)+x(8)*T^2*t.^2-2*x(8)*T*t.^3+x(8)*t.^4]';        
        ydot = [2*x(5)*T^2*t-6*x(5)*T*t.^2+4*x(5)*t.^3;
                2*x(6)*T^2*t-6*x(6)*T*t.^2+4*x(6)*t.^3;
                2*x(7)*T^2*t-6*x(7)*T*t.^2+4*x(7)*t.^3;
                2*x(8)*T^2*t-6*x(8)*T*t.^2+4*x(8)*t.^3;]';
        
    end

% alculate f1dot and f2dot by numerical differentiation
% (repeat last entry to maintain length
f1dot = diff([tau1 tau1(end)])/delta;
f2dot = diff([tau2 tau2(end)])/delta;

% calculate muscle state trajectories

for i = 1:length(t)
    
    % tendon forces
    Phi(:,i) = Di*[tau1(i);tau2(i);y3(i);y4(i);y5(i);y6(i)];
    
    % series element lengths
    LS(:,i) = SEEinv(Phi(:,i),Ls,k);
  
    % LS rates 
    denom = SEEPrime(LS(:,i),Ls,k);
    num = Di*[f1dot(i);f2dot(i);ydot(i,:)'];
    LSdot(:,i) = num./denom;
    
    % LC, u and a
    L(:,i) = a0-d1*q1(1,i)-d2*q2(1,i);
    Ldot(:,i) = -d1*q1(2,i)-d2*q2(2,i);
    LC(:,i) = L(:,i)-LS(:,i);
    u(:,i) = LSdot(:,i)-Ldot(:,i);
    FP(:,i) = PE(LC(:,i),Lo);
    act(:,i) = (Phi(:,i)-FP(:,i))./(f1(LC(:,i),W,Lo).*g1(u(:,i),Vm).*Fmax);
    
end

% if negative tendon force detected, increase FTreserve
if sum(sum(Phi<=0)) ~= 0
    worstCase = floor(min(min(Phi)));
    FTreserve = FTreserve + abs(worstCase)';
    continue
end

% obtain activation derivatives using diff (repeat last entry to maintain length)
act = [act act(:,end)];

a1dot = diff(act(1,:))/delta;
a2dot = diff(act(2,:))/delta;
a3dot = diff(act(3,:))/delta;
a4dot = diff(act(4,:))/delta;
a5dot = diff(act(5,:))/delta;
a6dot = diff(act(6,:))/delta;


% solve for neural inputs (based on saturation-type activation function)

for j = 1:length(t)
    [n,solfound] = solvesat(tau_act,beta_act,act(1,j),a1dot(j));
    n1(j) = n;
    [n,solfound] = solvesat(tau_act,beta_act,act(2,j),a2dot(j));
    n2(j) = n;
    [n,solfound] = solvesat(tau_act,beta_act,act(3,j),a3dot(j));
    n3(j) = n;
    [n,solfound] = solvesat(tau_act,beta_act,act(4,j),a4dot(j));
    n4(j) = n;
    [n,solfound] = solvesat(tau_act,beta_act,act(5,j),a5dot(j));
    n5(j) = n;
    [n,solfound] = solvesat(tau_act,beta_act,act(6,j),a6dot(j));
    n6(j) = n;
    
    % if negative n detected, increase FTreserve
    if n1(j)<0 || n2(j)<0 || n3(j)<0 || n4(j)<0 || n5(j)<0 || n6(j)<0
        FTreserve = FTreserve + 10;
        flag = 1;
        break
    else
        flag = 0;
    end
    
end

if flag == 1
    continue
else
    break
end

end

% if solution requires an FTreserve larger than the smallest muscle maximum
% isometric force or any of the n are without a solution, throw an error
if sum(FTreserve >= min(Fmax))~=0 || ~exist('n1') || ~exist('n2') || ~exist('n3') || ~exist('n4') || ~exist('n5') || ~exist('n6') 
    error('Solver failed:');
end

% return optimal n sequence and state predictions
n = [n1' n2' n3' n4' n5' n6'];

t = t';

end