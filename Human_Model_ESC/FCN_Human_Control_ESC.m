function FCN_Human_Control_ESC(robotpars,model_num,Traj_Opt,Imp_Opt)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%                                                                 %%%%%
%%%%%                        HUMAN  CONTROLLER                        %%%%%
%%%%%                                                                 %%%%%
%%%%%                     Humberto J De las Casas                     %%%%%
%%%%%                                                                 %%%%%
%%%%%                                                                 %%%%%
%%%%% Using the human model from:                                     %%%%%
%%%%% Holly Warner. "Simulation and Control at the Boundaries Between %%%%%
%%%%% Humans and Assistive Robots." Cleveland State University, 2019. %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Ellipse and circle (impedance) parameters:
Ellx = 0.25;
Elly = 0.15;
Cir  = 0.12;
Trx  = 0;
Try  = 0.3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Subject parameters:
r_par=[robotpars.m1, robotpars.m2, robotpars.l1, robotpars.lc1,...
       robotpars.l2, robotpars.lc2, robotpars.I1, robotpars.I2 ];
Rl1 = robotpars.l1;
Rl2 = robotpars.l2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Impedance parameters:
I  = [1;1];  %100 1 50
B  = [1;1];
K0 = [5;5];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Time parameters:
Theta0 = 0*pi/2;
Revs=600;
SpR = 32;           % Samples per revolution.
dT=2*pi/SpR;
f=1;                % Period of 2pi=1hz
T=dT:dT:(2*pi*Revs);
Theta_rev = 0.5;
Per_rev = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% ESC parameters:
omega1=0.1;
omega2=0.25;
a1=0.01;
a2=0.01;
k1=30;                              % +:Min. -:Max.
k2=3000;
S=[a1*sin(omega1*T);a2*sin(omega2*T)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Cartesian motor (zero-impedance) trajectories:
Mx=[Cir*cos(f*T)+Trx;-Cir*sin(f*T);-Cir*cos(f*T)];
My=[Cir*sin(f*T)+Try; Cir*cos(f*T);-Cir*sin(f*T)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Prelocation and initial conditions (PIC):
Rq1=zeros(3,length(T));Rq2=zeros(3,length(T));
Rx_arr=zeros(3,length(T));Ry_arr=zeros(3,length(T));
u=zeros(6,length(T));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Optimization mods and PIC2:
MA_Filt = 1500;
conv_th = [0.01,0.75];
vars=2;
conv_values = [inf,inf];
y = zeros(vars,length(T));
y(:,1) = [Theta0;mean(K0)];
if (Traj_Opt==1)&&(Imp_Opt==0)
    K = mean(K0);
    S(2,:) = 0;    
    conv_values(1,2) = 0;
elseif (Traj_Opt==0)&&(Imp_Opt==1)
    Theta = Theta0;
    S(1,:) = 0;
    conv_values(1,1) = 0;
end
Thetai = zeros(length(T),1);
Theta_arr = zeros(length(T),1);
Ki = zeros(length(T),1);
K_arr = zeros(length(T),1);
M_Per = zeros(length(T),1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Muscle weights:
%%%%% M1: Anterior Deltoid.
%%%%% M2: Posterior Deltoid.
%%%%% M3: Biceps Brachii.
%%%%% M4: Triceps Brachii (longhead).
%%%%% M5: Triceps Brachii (shorthead).
%%%%% M6: Brachialis.
W = ones(1,6); W(1)=-1; W(6)=-1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Simulation:
tic
i_break = inf;
for i=1:1:length(T)
    
    if (0==mod(i,100))&&(i>(MA_Filt+2))
        max(MA_conv_values);
    end
    
    if (Traj_Opt==1) 
        Thetai(i) = y(1,i)+S(1,i);
        if i == 1
            Theta_arr(i) = Thetai(i);
            Theta = Theta_arr(i);
            conv_values(i,1) = abs(Theta_arr(i));
        elseif i <= round(Theta_rev*8*pi/dT/omega1)
            Theta_arr(i) = mean(Thetai(1:i));
            Theta = Theta_arr(i);
            conv_values(i,1) = abs(mean(Thetai(1:i)));
        else
            Theta_arr(i) = mean(Thetai(i-round(Theta_rev*2*pi/dT/omega1):i));
            Theta = Theta_arr(i);
            conv_values(i,1) = abs(mean(Thetai(i-round(Theta_rev*4*pi/dT/omega1):i))-...
                                 mean(Thetai(i-round(Theta_rev*8*pi/dT/omega1):i-round(Theta_rev*4*pi/dT/omega1))));
        end
    end
    
    if (Imp_Opt == 1) 
        Ki(i) = y(2,i)+S(2,i);
        if i == 1
            K_arr(i) = Ki(i);
            K = K_arr(i);
            conv_values(i,2) = abs(Ki(i));
        elseif i <= round(Theta_rev*8*pi/dT/omega2)
            K_arr(i) = mean(Ki(1:i));
            K = K_arr(i);
            conv_values(i,2) = abs(mean(Ki(1:i)));
        else
            K_arr(i) = mean(Ki(i-round(Theta_rev*2*pi/dT/omega2):i));
            K = K_arr(i);
            conv_values(i,2) = abs(mean(Ki(i-round(Theta_rev*4*pi/dT/omega2):i))-...
                                 mean(Ki(i-round(Theta_rev*8*pi/dT/omega2):i-round(Theta_rev*4*pi/dT/omega2))));
        end
    end
    
    Rx=-Ellx*cos(f*T(i))*cos(f*Theta)-Elly*sin(f*T(i))*sin(f*Theta)+Trx;
    Ry=-Ellx*cos(f*T(i))*sin(f*Theta)+Elly*sin(f*T(i))*cos(f*Theta)+Try;
    
    [Rq1(1,i),Rq2(1,i)] = IKin (Rl1,Rl2,Rx,Ry);
    Rx_arr(1,i)=Rx; Ry_arr(1,i)=Ry;
    
    if i>1
        Rx_arr(2,i)=diff(Rx_arr(1,i-1:i))/dT;
        Ry_arr(2,i)=diff(Ry_arr(1,i-1:i))/dT;
        Rx_arr(3,i)=diff(Rx_arr(2,i-1:i))/dT;
        Ry_arr(3,i)=diff(Ry_arr(2,i-1:i))/dT;
    end
    xtilde = [Rx_arr(1,i)-Mx(1,i); Ry_arr(1,i)-My(1,i)];
    xtildeD = [Rx_arr(2,i)-Mx(2,i);Ry_arr(2,i)-My(2,i)];
    xtildeDD = [Rx_arr(3,i)-Mx(3,i);Ry_arr(3,i)-My(3,i)];
    Fext = -(I.*xtildeDD + B.*xtildeD + K.*xtilde)';

    u(:,i) = SOSflatOPT(Rq1(:,i),Rq2(:,i),dT,Fext,r_par);
    if  i == 1
        M_Per(i)=W*u(:,1:i);
    elseif i <= round(2*pi/dT/f*Per_rev)
        M_Per(i)=W*mean(u(:,1:i)')';
    else
        M_Per(i)=W*mean(u(:,i-round(2*pi/dT/f*Per_rev):i)')';
    end
    
    [~,y0] = ode15s(@(t,y) -M_Per(i)*[k1*S(1,i);k2*S(2,i)],...
             [T(i),T(i)+dT],y(:,i));
    y(:,i+1) = y0(end,:);
    
    if i>(MA_Filt+1)
        MA_conv_values = mean(conv_values(i-MA_Filt:i,:)); 
        if (max(MA_conv_values)<conv_th(Traj_Opt+Imp_Opt))
            if i_break>i
                i_break=i;
            end
        end
    end
    
    if i>i_break+round(Theta_rev*75*pi/dT/min(omega1,omega2))
        break
    end
    
end
toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% If it converged:
if i < length(T)
    K_arr(i+1:length(T))=[];
    Thetai(i+1:length(T))=[];
    Ki(i+1:length(T))=[];
    M_Per(i+1:length(T))=[];
    Theta_arr(i+1:length(T))=[];
    T(i+1:length(T))=[];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Final mods:
Theta=mean(Theta_arr(end-round(2*pi/dT/omega1)+1:end));
Theta=Theta*180/pi;
K=mean(K_arr(end-round(2*pi/dT/omega2)+1:end));
if (Traj_Opt==1)&&(Imp_Opt==0)
    K = mean(K0);
    K_arr = ones(length(K_arr),1).*K;
    Ki = ones(length(K_arr),1).*K;
end
DATA=[Theta,K];

if (Traj_Opt==1)&&(Imp_Opt==0)
    save (['ESC_Optimization_Model_',num2str(model_num),'_SV_T','.mat'],...
       'DATA','K_arr','Thetai','Ki','M_Per','Theta_arr','T');
elseif (Traj_Opt==0)&&(Imp_Opt==1)
    save (['ESC_Optimization_Model_',num2str(model_num),'_SV_I','.mat'],...
       'DATA','K_arr','Thetai','Ki','M_Per','Theta_arr','T');
else
    save (['ESC_Optimization_Model_',num2str(model_num),'_MV','.mat'],...
       'DATA','K_arr','Thetai','Ki','M_Per','Theta_arr','T');
end

   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Inverse kinematics algorithm:
function [q1,q2] = IKin (l1,l2,x,y)

    gamma = atan2(y,x);
    beta = acos((l1^2+l2^2-x.^2-y.^2)/(2*l1*l2));
    alpha = acos((x.^2+y.^2+l1^2-l2^2)./(2*l1*sqrt(x.^2+y.^2)));
    
    q1 = gamma - alpha;
    q2 = pi - beta;
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Main algorithms:
function yprime=Newton_Run(y,k,gamma,S,M,Theta)

    uhat=y(1:2);
    ustar=[1.5,1]';

    H=[10 -1;-1 30];

    Gv=y(3:5);
    G=[Gv(1),Gv(2);Gv(2),Gv(3)];

    u=uhat+S;

    yy=4+1/2*(u-ustar)'*H*(u-ustar);

    dGdt=gamma*(G-G*(yy*Theta)*G);

    duhatdt=-k*G*M*yy;
    yprime=[duhatdt;dGdt(1,1);dGdt(1,2);dGdt(2,2)];

end

end