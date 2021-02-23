% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Generates ellipse trajectories and evaluates any
% endpoint force resulting from impedance control being applied along those
% trajectories

levinTraj = 0;
displayPlots = 0;
% clean up assigned variables from a possible previous optimization trial
xMd=[];
yMd=[];
xMa=[];
yMa=[];
xEllA=[];
yEllA=[];
q1Md=[];
q2Md=[];
q1Ma=[];
q2Ma=[];
q1EllA=[];
q2EllA=[];

% define exercise machine parameters
% exercise machine link lengths
l1M = 0.5;
l2M = 0.5;

% exercise machine origin
oxM = 0;
oyM = 1;

% location of human arm origin (shoulder)
oxA = 0;
oyA = 0;

% determine trajectory from optimization solution
if useTrajOpt
    
    % machine actual trajectory parameters
    aMa = Population(popindex).chrom(1);
    bMa = Population(popindex).chrom(2);
    xcMa = Population(popindex).chrom(3);
    ycMa = Population(popindex).chrom(4);
    
    % machine desired trajectory parameters
    aMd = Population(popindex).chrom(6);
    bMd = Population(popindex).chrom(7);
    xcMd = Population(popindex).chrom(8);
    ycMd = Population(popindex).chrom(9);

    % direction, orientations, and angular frequency
    dir = Population(popindex).chrom(12); % +1 is counterclockwise, -1 is clockwise
    phiMa = Population(popindex).chrom(5);
    phiMd = Population(popindex).chrom(10);
    omega = Population(popindex).chrom(11)*2*pi;
    
% otherwise, use trajectory based on Levin 2001
else
    
    % machine actual trajectory parameters
    aMa = 5e-3;
    bMa = 0.18/2;
    xcMa = 0;
    ycMa = 0.4;
    
    % machine desired trajectory parameters
    aMd = aMa;
    bMd = bMa;
    xcMd = xcMa;
    ycMd = ycMa;
    
    % direction, orientations, and angular frequency
    dir = 1; % +1 is counterclockwise, -1 is clockwise
    if levinTraj == 1
        phiMd = pi;
    elseif levinTraj == 2
        phiMd = -pi/4;
    elseif levinTraj == 3
        phiMd = pi/2;
    elseif levinTraj == 4
        phiMd = pi/4;
    else
        error('Please specify Levin trajectory')
    end
    phiMa = phiMd;
    omega = 126/60*2*pi;

end

% time vector for trajectory
delta = 0.01;
T = 1/(omega/(2*pi));
tFit = [0:delta:T];

% impedance parameters
% determine trajectory from optimization solution
if useTrajOpt
    
    I = diag([Population(popindex).chrom(13),Population(popindex).chrom(14)]);
    B = diag([Population(popindex).chrom(15),Population(popindex).chrom(16)]);
    K = diag([Population(popindex).chrom(17),Population(popindex).chrom(18)]);
    
% otherwise, use trajectory based on Levin 2001
else
    
    I = 0*diag([1 1]);
    B = 0*diag([1 1]);
    K = 0*diag([1 1]);
    
end

% inverse kinematics of "machine desired" and "machine actual" ellipses
[xMd,yMd,q1Md,q2Md] = inverseKinEllipse(tFit,aMd,bMd,dir,omega,phiMd,xcMd,ycMd,oxM,oyM,l1M,l2M);
[xMa,yMa,q1Ma,q2Ma] = inverseKinEllipse(tFit,aMa,bMa,dir,omega,phiMa,xcMa,ycMa,oxM,oyM,l1M,l2M);

% compute forces being applied to machine endpoint by task space impedance control
% assuming perfect tracking
Fext = impedanceCtrlTaskSpace(xMa,yMa,xMd,yMd,I,B,K);

% plot scaled force vectors along trajectory
if displayPlots == 1
    figure();
    plot(xMa(1,:),yMa(1,:),'--k','LineWidth',1);
    hold on;
    quiver(xMa(1,1:4:end),yMa(1,1:4:end),Fext(1,1:4:end),Fext(2,1:4:end),'LineWidth',1)
    axis equal
    legend('Actual Trajectory','Scaled $F_{ext}$')
    xlabel('$x$ (m)');
    ylabel('$y$ (m)');
    set(findall(gcf,'-property','FontSize'),'FontSize',11)
    set(findall(gcf,'-property','FontName'),'FontName','Times New Roman')
    set(findall(gcf,'-property','Interpreter'),'Interpreter','latex');
end

% inverse kinematics to get arm joint trajectories
[xEllA,yEllA,q1EllA,q2EllA] = inverseKinEllipse(tFit,aMa,bMa,dir,omega,phiMa,xcMa,ycMa,oxA,oyA,robotpars.l1,robotpars.l2);

% check reachability
if ~isreal(q1Md) || ~isreal(q2Md) || ~isreal(q1Ma) || ~isreal(q2Ma)...
    || ~isreal(q1EllA) || ~isreal(q2EllA) || sum(q1EllA(1,:) < -30*pi/180) ~= 0 ...
    || sum(q2EllA(1,:) < 5*pi/180) ~= 0 || sum(diff(q1EllA(1,:)) > 5) ~= 0 ...
    || sum(diff(q2EllA(1,:)) > 5) ~= 0
    error('Trajectory not achievable')
end

% compute torques generated at arm joints by application of Fext
for idx = 1:length(q1EllA)
    tauA(:,idx) = JvEval([q1EllA(1,idx);q2EllA(1,idx)],robotpars.l1,robotpars.l2)'*Fext(:,idx);
end

% plot torque for 2 cycles to check "continuity"
if displayPlots == 1
    figure();plot(tFit,tauA','.');hold on;plot(tFit+T,tauA,'.')
end

% animations
if showAnimations == 1
    animHdl = figure();
    animColorMD = 'r';
    animColorMA = 'k';
    animColorA = 'b';
    decFctr = 10;
    animate(q1Ma(1,:),q2Ma(1,:),tFit(1:end-1),decFctr,xMa(1,:),yMa(1,:),l1M,l2M,oxM,oyM,animHdl,animColorMA)
    animate(q1Md(1,:),q2Md(1,:),tFit(1:end-1),decFctr,xMd(1,:),yMd(1,:),l1M,l2M,oxM,oyM,animHdl,animColorMD)
    animate(q1EllA(1,:),q2EllA(1,:),tFit(1:end-1),decFctr,xEllA(1,:),yEllA(1,:),robotpars.l1,robotpars.l2,oxA,oyA,animHdl,animColorA)
    xlabel('$x$ (m)')
    ylabel('$y$ (m)')
    children = get(gca, 'children');
    for childIdx = 1:6
        children(childIdx).LineWidth=1;
    end
    delete(children(2))
    children(4).LineStyle='--';
    children(6).LineStyle='--';
    axis([-0.7 0.7 -0.2 1.2])
    set(get(get(children(4),'Annotation'),'LegendInformation'),'IconDisplayStyle','off')
    set(get(get(children(6),'Annotation'),'LegendInformation'),'IconDisplayStyle','off')
    legend('Machine Actual','Machine Desired','Human Arm','Location','northeastoutside')
    set(findall(gcf,'-property','FontSize'),'FontSize',11)
    set(findall(gcf,'-property','FontName'),'FontName','Times New Roman')
    set(findall(gcf,'-property','Interpreter'),'Interpreter','latex');

end