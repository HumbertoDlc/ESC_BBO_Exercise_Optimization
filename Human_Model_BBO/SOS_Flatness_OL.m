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
% File description: Solves an optimal open-loop problem for the 2 dof, 6 muscle arm model.
% Resulting neural input histories can then applied to a forward dynamic
% simulation for verification.
global Person_Model
%%%load settings
global useSOSTools useTrajOpt

%%%parameters

% gravity switch
g = 0*9.81; % m/s^2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% linkage constants:
Model_1 = [1.91591346720137,1.71752064827779,0.289300000000000,0.131916568590110,0.325800000000000,0.229717816765050,0.0219157838022994,0.0389093377979930];
Model_2 = [1.97666374242890,1.78760439209137,0.293500000000000,0.131275694402840,0.327700000000000,0.221409693173921,0.0236447395330096,0.0420778095826136];
Model_3 = [2.11236529174690,1.78349621169084,0.296300000000000,0.126186371740153,0.329200000000000,0.232911060645160,0.0219827112490021,0.0391775873636525];
Model_4 = [2.04231677088206,1.89571144395262,0.300200000000000,0.129502804029635,0.331700000000000,0.219975848022404,0.0219495442259473,0.0426548052039723];
Model_5 = [2.10217050238507,1.82808862248585,0.306300000000000,0.135938727075232,0.336300000000000,0.235690154070172,0.0230617350885888,0.0422730337641185];
Model_6 = [2.04061228744266,1.95018359287662,0.310400000000000,0.138401481168882,0.339900000000000,0.224501214340154,0.0236525595813325,0.0424817308450313];
Model_7 = [2.22567022219423,1.89111225061403,0.313700000000000,0.138612179081824,0.342900000000000,0.237760406393924,0.0251210230669233,0.0409554136118972];
Model_8 = [2.12792388203448,1.86779577579633,0.316600000000000,0.140846993889732,0.345600000000000,0.244180809397556,0.0242490642113718,0.0438680028621966];
Model_9 = [2.12781322477666,1.92277872954213,0.319200000000000,0.137590152033621,0.348000000000000,0.245951308938509,0.0249175838049435,0.0442534179459066];
Model_10 = [2.09632050795284,1.88724401088193,0.321700000000000,0.144131981394084,0.350200000000000,0.232596979926169,0.0234848438509413,0.0429771924960821];

if Person_Model == 1
% Person 1:
robotpars.m1 = Model_1(1);
robotpars.m2 = Model_1(2);
robotpars.l1 = Model_1(3);
robotpars.lc1 = Model_1(4); % distance to center of mass
robotpars.l2 = Model_1(5);
robotpars.lc2 = Model_1(6); % distance to center of mass
robotpars.I1 = Model_1(7);
robotpars.I2 = Model_1(8);
elseif Person_Model == 2
% Person 2:
robotpars.m1 = Model_2(1);
robotpars.m2 = Model_2(2);
robotpars.l1 = Model_2(3);
robotpars.lc1 = Model_2(4); % distance to center of mass
robotpars.l2 = Model_2(5);
robotpars.lc2 = Model_2(6); % distance to center of mass
robotpars.I1 = Model_2(7);
robotpars.I2 = Model_2(8);
elseif Person_Model == 3
% % Person 3:
robotpars.m1 = Model_3(1);
robotpars.m2 = Model_3(2);
robotpars.l1 = Model_3(3);
robotpars.lc1 = Model_3(4); % distance to center of mass
robotpars.l2 = Model_3(5);
robotpars.lc2 = Model_3(6); % distance to center of mass
robotpars.I1 = Model_3(7);
robotpars.I2 = Model_3(8);
elseif Person_Model == 4
% % Person 4:
robotpars.m1 = Model_4(1);
robotpars.m2 = Model_4(2);
robotpars.l1 = Model_4(3);
robotpars.lc1 = Model_4(4); % distance to center of mass
robotpars.l2 = Model_4(5);
robotpars.lc2 = Model_4(6); % distance to center of mass
robotpars.I1 = Model_4(7);
robotpars.I2 = Model_4(8);
elseif Person_Model == 5
% % Person 5:
robotpars.m1 = Model_5(1);
robotpars.m2 = Model_5(2);
robotpars.l1 = Model_5(3);
robotpars.lc1 = Model_5(4); % distance to center of mass
robotpars.l2 = Model_5(5);
robotpars.lc2 = Model_5(6); % distance to center of mass
robotpars.I1 = Model_5(7);
robotpars.I2 = Model_5(8);
elseif Person_Model == 6
% % Person 6:
robotpars.m1 = Model_6(1);
robotpars.m2 = Model_6(2);
robotpars.l1 = Model_6(3);
robotpars.lc1 = Model_6(4); % distance to center of mass
robotpars.l2 = Model_6(5);
robotpars.lc2 = Model_6(6); % distance to center of mass
robotpars.I1 = Model_6(7);
robotpars.I2 = Model_6(8);
elseif Person_Model == 7
% Person 7:
robotpars.m1 = Model_7(1);
robotpars.m2 = Model_7(2);
robotpars.l1 = Model_7(3);
robotpars.lc1 = Model_7(4); % distance to center of mass
robotpars.l2 = Model_7(5);
robotpars.lc2 = Model_7(6); % distance to center of mass
robotpars.I1 = Model_7(7);
robotpars.I2 = Model_7(8);
elseif Person_Model == 8
% Person 8:
robotpars.m1 = Model_8(1);
robotpars.m2 = Model_8(2);
robotpars.l1 = Model_8(3);
robotpars.lc1 = Model_8(4); % distance to center of mass
robotpars.l2 = Model_8(5);
robotpars.lc2 = Model_8(6); % distance to center of mass
robotpars.I1 = Model_8(7);
robotpars.I2 = Model_8(8);
elseif Person_Model == 9
% Person 9:
robotpars.m1 = Model_9(1);
robotpars.m2 = Model_9(2);
robotpars.l1 = Model_9(3);
robotpars.lc1 = Model_9(4); % distance to center of mass
robotpars.l2 = Model_9(5);
robotpars.lc2 = Model_9(6); % distance to center of mass
robotpars.I1 = Model_9(7);
robotpars.I2 = Model_9(8);
elseif Person_Model == 10
% Person 10:
robotpars.m1 = Model_10(1);
robotpars.m2 = Model_10(2);
robotpars.l1 = Model_10(3);
robotpars.lc1 = Model_10(4); % distance to center of mass
robotpars.l2 = Model_10(5);
robotpars.lc2 = Model_10(6); % distance to center of mass
robotpars.I1 = Model_10(7);
robotpars.I2 = Model_10(8);
end

% kinematics of muscle attachment
d1 = [0.05;-0.05;0.03;-0.03;0;0];
d2 = [0;0;0.03;-0.03;-0.03;0.03];

Scale_Link_1 = robotpars.l1/0.33;
Scale_Link_2 = robotpars.l2/0.32;
a0 = [0.1840*Scale_Link_1;0.1055*Scale_Link_1;...
      0.4283*Scale_Link_1;0.1916*Scale_Link_1;...
      0.2387*Scale_Link_2;0.1681*Scale_Link_2];
Ls = [0.0538*Scale_Link_1;0.0538*Scale_Link_1;...
      0.2298*Scale_Link_1;0.1905*Scale_Link_1;...
      0.1905*Scale_Link_2;0.0175*Scale_Link_2];
Lo = [0.1280*Scale_Link_1 0.1280*Scale_Link_1...
      0.1422*Scale_Link_1 0.0877*Scale_Link_1...
      0.0877*Scale_Link_2 0.1028*Scale_Link_2]';

% maximum contraction velocity
Vm = 10*Lo;

% width of force-length curve
W  = 0.56*ones(6,1);

% maximum isometric muscle forces
Fmax = [800;800;1000;1000;700;700];

% slope of tendon stiffness
k = Fmax./(0.04*Ls).^2;

% collect all muscle parameters
musclepars.d1 = d1;
musclepars.d2 = d2;
musclepars.a0 = a0;
musclepars.Ls = Ls;
musclepars.Lo = Lo;
musclepars.Vm = Vm;
musclepars.W = W;
musclepars.Fmax = Fmax;
musclepars.k = k;

% define activation dynamics time constant
tau_act = 0.01;  
beta_act = 0.25;
tau_deact = tau_act/beta_act;

%%%trajectories
trajMain

%%%initial conditions
q0 = [q1EllA(1,1);q2EllA(1,1)];
qdot0 = [q1EllA(2,1);q2EllA(2,1)];
z0 = [q0;qdot0];

% qddot0 = [q1EllA(3,1);q2EllA(3,1)];

x0 = [z0;zeros(6,1);zeros(6,1)];
x = x0;

%%%flatness-SOS optimizer
% tic
[t,n,act,LS,u,Phi,q1pred,q2pred,FTreserve] = SOSflatOPT(q1EllA,q2EllA,robotpars,musclepars,g,T,delta,tau_act,beta_act,tauA,useSOSTools);
% toc

%%%post-process to find co-contraction histories

% co-contraction averages matrix
E = [0.5 0.5 0 0 0 0;0 0 0.5 0.5 0 0;0 0 0 0 0.5 0.5;1/3 0 1/3 0 1/3 0];

% compute co-contraction flat outputs
Y36 = E*Phi;

%%%forward integration using the optimal neural controls for verification
n_time = t;
z0 = [q1pred(1,1);q2pred(1,1);q1pred(2,1);q2pred(2,1)];
x0 = [z0;LS(:,1);act(:,1)];

if forwardSim == 1
    % set options
    % if integration diverges, might need to tighten tolerances
    atol_ode  = 1e-6;
    rtol_ode  = 1e-6;
    % use event function to interrupt part-way through integration. See
    % pauseFunc() for details
    options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode,'Events',@(t,y)pauseFunc(t,y,omega));
    [TOUT,YOUT] = ode23s(@(t,x) mdlStateDer(t,x,tauA,n_time,n,robotpars,musclepars,g,tau_act,beta_act),[0  T],x0,options);
end

%%%%plot results
if displayPlots == 1
    
% predicted and forward-simulated trajectories
figure()
subplot(2,1,1);
plot(t,q1pred(1,:),'+k');hold on
if forwardSim == 1
plot(TOUT,YOUT(:,1),'k','LineWidth',2)
end
legend('q (optimal planning)','q (forward integration)')
ylabel('$q_1$', 'Fontsize',14,'Interpreter','Latex')
title('Open-Loop Optimization: Planned and Actual Joint Trajectories','FontSize',14,'Interpreter','Latex')

subplot(2,1,2);
plot(t,q2pred(1,:),'+k');hold on
if forwardSim == 1
plot(TOUT,YOUT(:,2),'k','LineWidth',2)
end
ylabel('$q_2$', 'Fontsize',14,'Interpreter','Latex')
xlabel('Time,s' ,'Fontsize',14,'Interpreter','Latex')

% tendon lengths
figure()
subplot(3,2,1)
plot(t,LS(1,:),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,5),'k')
end
legend('LS (optimal planning)','LS (forward integration)')
ylabel('$LS_1$', 'Fontsize',14,'Interpreter','Latex')
title('Open-Loop Optimization: SE Lengths','FontSize',14,'Interpreter','Latex')

subplot(3,2,2)
plot(t,LS(2,:),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,6),'k')
end
ylabel('$LS_2$', 'Fontsize',14,'Interpreter','Latex')

subplot(3,2,3)
plot(t,LS(3,:),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,7),'k')
end
ylabel('$LS_3$', 'Fontsize',14,'Interpreter','Latex')

subplot(3,2,4)
plot(t,LS(4,:),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,8),'k')
end
ylabel('$LS_4$', 'Fontsize',14,'Interpreter','Latex')

subplot(3,2,5)
plot(t,LS(5,:),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,9),'k')
end
ylabel('$LS_5$', 'Fontsize',14,'Interpreter','Latex')
xlabel('Time,s' ,'Fontsize',14,'Interpreter','Latex')

subplot(3,2,6)
plot(t,LS(6,:),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,10),'k')
end
ylabel('$LS_6$', 'Fontsize',14,'Interpreter','Latex')
xlabel('Time,s' ,'Fontsize',14,'Interpreter','Latex')

% muscle activations and neural inputs
if useTrajOpt == 0 && levinTraj == 3
    act = act(:,1:end-1);
elseif useTrajOpt == 0 && levinTraj == 4
    act = act(:,1:end-2);
end
figure()
subplot(3,2,1)
plot(t,act(1,1:end-1),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,11),'k','LineWidth',2)
end
plot(t,n(:,1),'--k')
axis([0 inf 0 1])
ylabel('$n_1,a_1$', 'Fontsize',14,'Interpreter','Latex')
title('Open-Loop Optimization: Activations and Neural Inputs','FontSize',14,'Interpreter','Latex')


subplot(3,2,2)
plot(t,act(2,1:end-1),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,12),'k','LineWidth',2)
end
plot(t,n(:,2),'--k')
axis([0 inf 0 1])
ylabel('$n_2,a_2$', 'Fontsize',14,'Interpreter','Latex')
legend('a (optimal planning)','a (forward integration)','n')

subplot(3,2,3)
plot(t,act(3,1:end-1),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,13),'k','LineWidth',2);
end
plot(t,n(:,3),'--k')
axis([0 inf 0 1])
ylabel('$n_3,a_3$', 'Fontsize',14,'Interpreter','Latex')

subplot(3,2,4)
plot(t,act(4,1:end-1),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,14),'k','LineWidth',2)
end
plot(t,n(:,4),'--k')
axis([0 inf 0 1])
ylabel('$n_4,a_4$', 'Fontsize',14,'Interpreter','Latex')

subplot(3,2,5)
plot(t,act(5,1:end-1),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,15),'k','LineWidth',2)
end
plot(t,n(:,5),'--k')
axis([0 inf 0 1])
ylabel('$n_5,a_5$', 'Fontsize',14,'Interpreter','Latex')
xlabel('Time,s' ,'Fontsize',14,'Interpreter','Latex')

subplot(3,2,6)
plot(t,act(6,1:end-1),'+k'); hold on
if forwardSim == 1
plot(TOUT,YOUT(:,16),'k','LineWidth',2)
end
plot(t,n(:,6),'--k')
axis([0 inf 0 1])
ylabel('$n_6,a_6$', 'Fontsize',14,'Interpreter','Latex')
xlabel('Time,s' ,'Fontsize',14,'Interpreter','Latex')

% co-contractions
figure()
subplot(4,1,1)
title('Open-Loop Optimization: Co-Contractions','FontSize',14,'Interpreter','Latex')
plot(t,Y36(1,:),'k-')
ylabel('$y_3$', 'Fontsize',14,'Interpreter','Latex')

subplot(4,1,2)
plot(t,Y36(2,:),'k-+')
ylabel('$y_4$', 'Fontsize',14,'Interpreter','Latex')

subplot(4,1,3)
plot(t,Y36(3,:),'k-o')
ylabel('$y_5$', 'Fontsize',14,'Interpreter','Latex')

subplot(4,1,4)
plot(t,Y36(4,:),'k-*')
ylabel('$y_6$', 'Fontsize',14,'Interpreter','Latex')
xlabel('Time,s' ,'Fontsize',14,'Interpreter','Latex')

% net torque, generalized muscle torque, and interactive torque for comparison
% to papers by Dounskaia et al.
figure()
subplot(2,1,1)
NTShoulder = (robotpars.m1*robotpars.lc1^2+robotpars.I1)*q1pred(3,:);
for idx = 1:length(LS)
    GMTShoulder(idx) = d1'*SEE(LS(:,idx),Ls,k);
end
ITShoulder = NTShoulder - GMTShoulder;
plot(t,NTShoulder,'.')
hold on;
plot(t,GMTShoulder)
plot(t,ITShoulder,'--')
legend('NT','GMT','IT')
title('Shoulder')

subplot(2,1,2)
NTElbow = (robotpars.m2*robotpars.lc2^2+robotpars.I2)*q2pred(3,:);
for idx = 1:length(LS)
    GMTElbow(idx) = d2'*SEE(LS(:,idx),Ls,k);
end
ITElbow = NTElbow - GMTElbow;
plot(t,NTElbow,'.')
hold on;
plot(t,GMTElbow)
plot(t,ITElbow,'--')
legend('NT','GMT','IT')
title('Elbow')

% replicate data for five cycles for comparison to paper by Levin et al. (2001)
tFiveCycles = [t;t(end)+delta+t;(t(end)+delta)*2+t;(t(end)+delta)*3+t;(t(end)+delta)*4+t];
actFiveCycles = repmat(act(:,1:end-1),[1 5]);

figure();
scale = 1;
subplot(2,1,1)
plot(tFiveCycles,scale*actFiveCycles(1,:),tFiveCycles,-scale*actFiveCycles(2,:))
hold on;
fill([tFiveCycles;flipud(tFiveCycles)],[scale*actFiveCycles(1,:) flipud(zeros(size(actFiveCycles(1,:))))],[0.7,0.7,0.7],'edgecolor','none')
fill([tFiveCycles;flipud(tFiveCycles)],[-scale*actFiveCycles(2,:) flipud(zeros(size(actFiveCycles(2,:))))],[0.5,0.5,0.5],'edgecolor','none')
plot(tFiveCycles,repmat(NTShoulder,[1 5]))
title('Shoulder')

scale = 1;
subplot(2,1,2)
plot(tFiveCycles,scale*(actFiveCycles(3,:)+actFiveCycles(6,:)),tFiveCycles,-scale*(actFiveCycles(4,:)))
hold on;
fill([tFiveCycles;flipud(tFiveCycles)],[scale*(actFiveCycles(3,:)+actFiveCycles(6,:)) flipud(zeros(size(actFiveCycles(3,:))))],[0.7,0.7,0.7],'edgecolor','none')
fill([tFiveCycles;flipud(tFiveCycles)],[-scale*(actFiveCycles(4,:)) flipud(zeros(size(actFiveCycles(4,:))))],[0.5,0.5,0.5],'edgecolor','none')
plot(tFiveCycles,repmat(NTElbow,[1 5]))
title('Elbow')

figure()
subplot(8,1,1)
hold on;
fill([tFiveCycles;flipud(tFiveCycles)],[actFiveCycles(1,:) flipud(zeros(size(actFiveCycles(1,:))))],[0.7,0.7,0.7],'edgecolor','none')
axis([0 inf 0 1])
ylabel({'Anterior';'Deltoid'})
subplot(8,1,2)
hold on;
fill([tFiveCycles;flipud(tFiveCycles)],[actFiveCycles(2,:) flipud(zeros(size(actFiveCycles(1,:))))],[0.7,0.7,0.7],'edgecolor','none')
axis([0 inf 0 1])
ylabel({'Posterior';'Deltoid'})
subplot(8,1,3)
hold on;
plot(tFiveCycles,repmat(NTShoulder,[1 5]))
axis([0 inf -inf inf])
ylabel('Shoulder NT')
subplot(8,1,4)
hold on;
fill([tFiveCycles;flipud(tFiveCycles)],[actFiveCycles(3,:) flipud(zeros(size(actFiveCycles(1,:))))],[0.7,0.7,0.7],'edgecolor','none')
axis([0 inf 0 1])
ylabel('Biceps Brachii')
subplot(8,1,5)
hold on;
fill([tFiveCycles;flipud(tFiveCycles)],[actFiveCycles(4,:) flipud(zeros(size(actFiveCycles(1,:))))],[0.7,0.7,0.7],'edgecolor','none')
axis([0 inf 0 1])
ylabel({'Triceps Brachii';'(Long head)'})
subplot(8,1,6)
hold on;
fill([tFiveCycles;flipud(tFiveCycles)],[actFiveCycles(5,:) flipud(zeros(size(actFiveCycles(1,:))))],[0.7,0.7,0.7],'edgecolor','none')
axis([0 inf 0 1])
ylabel({'Triceps Brachii';'(Short head)'})
subplot(8,1,7)
hold on;
fill([tFiveCycles;flipud(tFiveCycles)],[actFiveCycles(6,:) flipud(zeros(size(actFiveCycles(1,:))))],[0.7,0.7,0.7],'edgecolor','none')
axis([0 inf 0 1])
ylabel('Brachialis')
subplot(8,1,8)
hold on;
plot(tFiveCycles,repmat(NTElbow,[1 5]))
axis([0 inf -inf inf])
ylabel('Elbow NT')
xlabel('Time, s')

% replicate Levin 2001 plot setup
figure()
subplot(6,1,1)
hold on;
plot(tFiveCycles,repmat(xEllA(1,:)*100,[1 5]),tFiveCycles,repmat((yEllA(1,:)-ycMa)*100,[1 5]))
legend('$x$','$y$','interpreter','latex')
axis([0 inf -inf inf])
ylabel({'Displacements';'(cm)'})

subplot(6,1,2)
hold on;
tangAcc = diff(sqrt([xEllA(2,:) xEllA(2,end)].^2+[yEllA(2,:) yEllA(2,end)].^2)')'./delta;
plot(tFiveCycles,repmat(tangAcc*100,[1 5]))
axis([0 inf -inf inf])
ylabel({'Tangential';'Acceleration';'(cm/s$^2$)'},'Interpreter','latex')

subplot(6,1,3)
hold on;
plot(tFiveCycles,actFiveCycles(1,:))
axis([0 inf 0 1])
ylabel({'Anterior';'Deltoid';'Activation'})

subplot(6,1,4)
hold on;
plot(tFiveCycles,actFiveCycles(2,:))
axis([0 inf 0 1])
ylabel({'Posterior';'Deltoid';'Activation'})

subplot(6,1,5)
hold on;
plot(tFiveCycles,actFiveCycles(3,:))
axis([0 inf 0 1])
ylabel({'Biceps';'Brachii';'Activation'})

subplot(6,1,6)
hold on;
plot(tFiveCycles,actFiveCycles(5,:))
axis([0 inf 0 1])
ylabel({'Triceps Brachii';'(short head)';'Activation'})
xlabel('$t$ (s)','interpreter','latex')
end

%%%toggle to format and save plots as used in dissertation
if 0 
    
% assign date string manually here, should match that assigned by the
% optimizer in saving the reults, if it was used
dateStr = '00-00-0000_00-00';
    
% joint coordinates
figure()
subplot(2,1,1);
plot(t,q1pred(1,:)*180/pi,'LineWidth',1)
ylabel('$q_1$ (deg)')

subplot(2,1,2);
plot(t,q2pred(1,:)*180/pi,'LineWidth',1)
ylabel('$q_2$ (deg)')
xlabel('$t$ (s)' ,'Fontsize',14,'Interpreter','Latex')

set(findall(gcf,'-property','FontSize'),'FontSize',11)
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman')
set(findall(gcf,'-property','Interpreter'),'Interpreter','latex');

% activations
figure()
subplot(3,2,1)
plot(t,act(1,1:end-1),'LineWidth',1)
axis([0 inf 0 1])
ylabel('$a_1$')

subplot(3,2,2)
plot(t,act(2,1:end-1),'LineWidth',1)
axis([0 inf 0 1])
ylabel('$a_2$')

subplot(3,2,3)
plot(t,act(3,1:end-1),'LineWidth',1)
axis([0 inf 0 1])
ylabel('$a_3$')

subplot(3,2,4)
plot(t,act(4,1:end-1),'LineWidth',1)
axis([0 inf 0 1])
ylabel('$a_4$')

subplot(3,2,5)
plot(t,act(5,1:end-1),'LineWidth',1)
axis([0 inf 0 1])
ylabel('$a_5$')
xlabel('$t$ (s)')

subplot(3,2,6)
plot(t,act(6,1:end-1),'LineWidth',1)
axis([0 inf 0 1])
ylabel('$a_6$')
xlabel('$t$ (s)')

set(findall(gcf,'-property','FontSize'),'FontSize',11)
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman')
set(findall(gcf,'-property','Interpreter'),'Interpreter','latex');

% tendon force
figure()
plot(t,Phi','LineWidth',1)
xlabel('$t$ (s)')
ylabel('$\phi_S$ (N)')
legend('$\phi_{S1}$','$\phi_{S2}$','$\phi_{S3}$','$\phi_{S4}$','$\phi_{S5}$','$\phi_{S6}$','location','northeastoutside')

set(findall(gcf,'-property','FontSize'),'FontSize',11)
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman')
set(findall(gcf,'-property','Interpreter'),'Interpreter','latex');

% co-contraction flat outputs
figure()
plot(t,Y36(1,:),t,Y36(2,:),t,Y36(3,:),t,Y36(4,:),'LineWidth',1)
ylabel('$Y$ (N)')
xlabel('$t$ (s)')
legend('$y_3$','$y_4$','$y_5$','$y_6$','location','northeastoutside')

set(findall(gcf,'-property','FontSize'),'FontSize',11)
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman')
set(findall(gcf,'-property','Interpreter'),'Interpreter','latex');

% save plots
ID = {'Fext','Schematic','Traj','Act','PhiS','Y'};

ctr = 1;
for figIdx = [1 3 12:15]
    h = figure(figIdx);
    savefig(h,[dateStr,'_',ID{ctr},'.fig']);
    ctr = ctr+1;
end

end

function [value, isterminal, direction] = pauseFunc(T,Y,omega)
tStop = 2*pi/omega; % assign interrupt time here
value      = (T > tStop);
isterminal = 1;   % stop the integration
direction  = 0;
end