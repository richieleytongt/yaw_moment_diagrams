%% Preamble
tic
clc; clear; close all;
addpath('Functions'); addpath("Data"); addpath("Results");

OI.orange     = [230, 159, 0]   / 255;
OI.skyBlue    = [86, 180, 233]  / 255;
OI.blueGreen  = [0, 158, 115]   / 255;
OI.yellow     = [240, 228, 66]  / 255;
OI.blue       = [0, 114, 178]   / 255;
OI.vermillion = [213, 94, 0]    / 255;
OI.purple     = [204, 121, 167] / 255;

AdigatorRegen = true;

%% Define scaling and inputs

%Principle parameters
auxdata.g0 = 9.81;
auxdata.l0 = 3.4;
auxdata.m0 = 720;

% scale factors
auxdata.lengthscale  = 1/auxdata.l0;                 lengthscale  = auxdata.lengthscale;
auxdata.massscale    = 1/auxdata.m0;                 massscale    = auxdata.massscale;
auxdata.timescale    = sqrt(auxdata.g0./auxdata.l0); timescale    = auxdata.timescale;
auxdata.velscale     = lengthscale/timescale;        velscale     = auxdata.velscale;
auxdata.accscale     = lengthscale/timescale^2;      accscale     = auxdata.accscale;
auxdata.forcescale   = massscale*accscale;           forcescale   = auxdata.forcescale;
auxdata.powerscale   = forcescale*velscale;          powerscale   = auxdata.powerscale;
auxdata.inertiascale = massscale*lengthscale^2;      inertiascale = auxdata.inertiascale;
auxdata.springscale  = forcescale/lengthscale;       springscale  = auxdata.springscale;
auxdata.dampscale    = massscale/timescale;          dampscale    = auxdata.dampscale;
auxdata.g            = auxdata.g0*accscale;

run('bicycle.m');       %Car data
auxdata.rho = 1.21094734506627*massscale/lengthscale^3; %air density (kg/m^3)

%road profile
auxdata.k   = 0;          %road inclination (positive uphill)
auxdata.phi = 0.0*pi/180; %road banking (negative left NASCAR friendly)

%motion characteristics
auxdata.V  = 50.0*velscale; %whole vehicle velocity (m/s)
auxdata.ax = 0.0*accscale;  %longitudinal acceleration (m/s^2)
auxdata.s0 = -2;   %initial negative lateral acceleration guess (g)
auxdata.sf = 2;    %initial positive lateral acceleration guess (g)

%Tire data
run('F12014_tire.m');
auxdata.tir.Fz_1 = auxdata.tir.Fz_1*forcescale;
auxdata.tir.Fz_2 = auxdata.tir.Fz_2*forcescale;

%to be organized
eps = 1.0e-4; auxdata.eps = eps;

%% Build 'auxdata' Structure

%non-dimensionalize and store planetary parameters
auxdata.g      = auxdata.g0*accscale;

%withdraw nondimensional terms
M   = auxdata.M;   %withdraw nondimensional mass from auxdata structure
g   = auxdata.g;   %withdraw nondimensional gravity from auxdata structure
rho = auxdata.rho; %withdraw nondimensional air density from auxdata structure
Af  = auxdata.Af;  %withdraw nondimensional frontal area from auxdata structure
h   = auxdata.h;   %withdraw nondimensional cg height from auxdata structure
a   = auxdata.a;   %withdraw nondimensional a
b   = auxdata.b;   %withdraw nondimensional b
k = auxdata.k;   %withdraw nondimensional road inclination from auxdata structure
phi = auxdata.phi; %withdraw nondimensional road banking from auxdata structure

%% Build 'bounds' Structure

%define state and input scales
auxdata.statescale = [1 1 1];                       % The states: beta, k_f, k_r
auxdata.inputscale = [1 1 1 forcescale forcescale];	% The controls: dbeta, dk_f, dk_r, Ffz, Frz;

%set limits on the states
initialStateLower = [-20*pi/180.0  -.1 -.1].*auxdata.statescale;
initialStateUpper = [ 20*pi/180.0    0  .1].*auxdata.statescale;
stateLower        = initialStateLower;
stateUpper        = initialStateUpper;
finalStateLower   = stateLower;
finalStateUpper   = stateUpper;

%set limits on the controls
controlLower      = [-5 -0.5 -0.5 -25000 -25000 ].*auxdata.inputscale;
controlUpper      = [ 5  0.5  0.5    0.0    0.0 ].*auxdata.inputscale;

%define maximum values of slip
max_alp = 8.0*pi/180;
max_k   = .08;

%store state, control, path and integral limits
bounds.phase.initialtime.lower  = auxdata.s0;
bounds.phase.initialtime.upper  = auxdata.s0;
bounds.phase.finaltime.lower    = auxdata.sf;
bounds.phase.finaltime.upper    = auxdata.sf;
bounds.phase.initialstate.lower = initialStateLower;
bounds.phase.initialstate.upper = initialStateUpper;
bounds.phase.state.lower        = stateLower;
bounds.phase.state.upper        = stateUpper;
bounds.phase.finalstate.lower   = finalStateLower;
bounds.phase.finalstate.upper   = finalStateUpper;
bounds.phase.control.lower      = controlLower;
bounds.phase.control.upper      = controlUpper;
bounds.phase.path.lower         = [-eps -eps -max_alp -max_k -max_alp -max_k];
bounds.phase.path.upper         = [+eps +eps +max_alp +max_k +max_alp +max_k];
bounds.phase.integral.lower     = 0;
bounds.phase.integral.upper     = 500;

%% Build 'guess' structure

T = linspace(-2,2,500)';
X = ones(500,3)*diag([0.01;0.01;0.01]);
U = ones(500,5)*diag([0.01;0.01;0.01;-10000*forcescale;-10000*forcescale]);

guess.phase.time             = T;
guess.phase.state            = X;
guess.phase.control          = U;
guess.phase.integral         = 1.0;

%% Build 'setup' structure
%name
setup.name                           = 'YMD car';               %name of the problem
setup.functions.continuous           = @BicycleYMDContinuous;   %name of the continuous function
setup.functions.endpoint             = @BicycleYMDEndpoint;     %name of the endpoint function
setup.nlp.solver                     = 'ipopt'; % 'ipopt'       %default solver: gradient descent algorithm
setup.nlp.ipoptoptions.linear_solver = 'ma57';                  
setup.nlp.ipoptoptions.maxiterations = 2000;                    %default value
setup.method                         = 'RPM-Differentiation';   %default method
setup.bounds                         = bounds;                  %deposit bounds structure into setup structure
setup.guess                          = guess;                   %deposit guess structure into setup structure
setup.derivatives.supplier           = 'adigator';              
if AdigatorRegen == false
    setup.adigatorgrd.continuous     = @BicycleYMDContinuousADiGatorGrd;
    setup.adigatorgrd.endpoint       = @BicycleYMDEndpointADiGatorGrd;
    setup.adigatorhes.continuous     = @BicycleYMDContinuousADiGatorHes;
    setup.adigatorhes.endpoint       = @BicycleYMDEndpointADiGatorHes;
end
setup.derivatives.dependencies       = 'sparseNaN';             %default
setup.derivatives.derivativelevel    = 'second';                %'snopt' -> 'first' (automatic)
setup.scales.method                  = 'automatic-bounds';      %comment out if no scaling either scaled yourself or properly scaled
setup.mesh.method                    = 'hp-PattersonRao';       %default
setup.mesh.tolerance                 = 1.0e-5;                  %maximum relative error over all the segments
setup.mesh.colpointsmin              = 4;                       
setup.mesh.colpointsmax              = 15;
setup.mesh.maxiterations             = 20; % max refinements
Numsec                               = 20;
setup.mesh.phase.colpoints           = 4.*ones(1,Numsec);       %default
setup.mesh.phase.fraction            = 1/Numsec.*ones(1,Numsec);

%% Execute GPOPS2 loop

%steering angles
deltaRef =  -9:1:9;                             %allocations for delta lines

%preallocations
AyVMin = zeros(1,length(deltaRef));
AyVMax = zeros(1,length(deltaRef));
MzMin = zeros(1,length(deltaRef));
MzMax = zeros(1,length(deltaRef));
TempMin = zeros(length(deltaRef),401);
TempMax = zeros(length(deltaRef),401);
ayMesh = zeros(length(deltaRef),401);
BetaMesh = zeros(length(deltaRef),401);
MzMesh = zeros(length(deltaRef),401);
xqP = zeros(length(deltaRef),15);
MzqP = zeros(length(deltaRef),15);


for kk = 1:length(deltaRef)
    auxdata.delta  = deltaRef(kk)*pi/180;
    delta          = auxdata.delta;
    setup.auxdata  = auxdata;
    output         = gpops2(setup);

    %withdraw states and controls
    ay          = output.result.interpsolution.phase.time;          %withdrawing lateral acceleration bounds from output structure
    beta        = output.result.interpsolution.phase.state(:,1);    %withdrawing beta vector
    kf          = output.result.interpsolution.phase.state(:,2);    %withdrawing front long slip vector
    kr          = output.result.interpsolution.phase.state(:,3);    %withdrawing rear long slip vector from output structure
    Ffz         = output.result.interpsolution.phase.control(:,4);  %withdrawing front downforce? from output structure
    Frz         = output.result.interpsolution.phase.control(:,5);  %withdrawing rear downforce? from output structure

    V           = auxdata.V;    %withdrawing non-dimensionalized velocity from auxdata structure
    phi         = auxdata.phi;  %withdrawing non-dimensionalized road banking
    ax          = auxdata.ax;   %withdrawing non-dimensionalized longitudinal acceleration

    u           = V.*cos(beta);                                                         %calculating longitudinal velocity
    v           = V.*sin(beta);                                                         %calculating lateral velocity
    R           = (V.^2 + sqrt(V.^4 - 4*k^2*ay.^2))./(2*cos(phi).*sqrt(ay.^2 + eps));   %calculating turn radius
    omega       = -sqrt(sqrt(ay.^2 + eps)./R./cos(phi)).*2/pi.*atan(1000*(ay+eps));     %calculating yaw rate
    omega_Bz    = omega.*R.*cos(phi).^2./sqrt(R.^2.*cos(phi).^2 + k.^2);                %

    %calculating front and rear slip angles
    alp_f   = atan2(v+omega_Bz.*a,u) - delta;
    alp_r   = atan2(v-omega_Bz.*b,u);

    %Tire force calculations
    [Ffx, Ffy] = Tire_force(alp_f, kf, Ffz, auxdata.tir);
    [Frx, Fry] = Tire_force(alp_r, kr, Frz, auxdata.tir);

    %YAW MOMENT CALCULATION
    Mz      = a.*(cos(delta).*Ffy + sin(delta).*Ffx) - b.*Fry;

    %aerodynamic forces
    Fax 	= 0.5*rho*Af*CD.*u.^2;
    Fay 	= 0.5*rho*Af*CS.*u.^2;
    Faz 	= 0.5*rho*Af*CL.*u.^2;

    %gravitational forces (banking)
    Gx      = -g./sqrt(cos(phi)^2.*R.^2+k^2).*(R.*sin(beta).*sin(2*phi)/2 - k*cos(beta));
    Gy      = g./sqrt(cos(phi)^2.*R.^2+k^2).*(R.*cos(beta).*sin(2*phi)/2 - k*sin(beta));
    Gz      = g./sqrt(cos(phi)^2.*R.^2+k^2).*(R.*cos(phi)^2);

    %chassis accelerations (banking)
    AxB     = ay.*cos(phi).*sin(beta);
    AyB     = -ay.*cos(phi).*cos(beta);
    AzB     = ay.*sin(phi);

    %chassis forces
    Fx 		= cos(delta).*Ffx - sin(delta).*Ffy + Frx + M*Gx - Fax;
    Fy 		= cos(delta).*Ffy + sin(delta).*Ffx + Fry + M*Gy + Fay;
    Fz 		= Frz + Ffz + M*Gz + Faz + AzB;

    %residues
    R1 	= ax.*cos(beta) -AxB - Fx./M;
    R2 	= ax.*sin(beta) -AyB - Fy./M;

    TsT = R1.^2 + R2.^2;
    valid = TsT' < 0.005;
    ii = [0, valid, 0];
    idxValid = find(valid);
    i1 = idxValid(1);
    i2 = idxValid(end);
    AyV = ay*cos(phi);
    AyVMin(kk) = AyV(i1);
    AyVMax(kk) = AyV(i2);
    MzMin(kk) = min(Mz(i1:i2));
    MzMax(kk) = max(Mz(i1:i2));

    % TempMin(kk,:) = Mz'.*ii(2:end-1) + [1*ones(1,i1-1) zeros(1,i2-i1) 1*ones(1,size(ii,2)-i2-1)];
    % TempMax(kk,:) = Mz'.*ii(2:end-1) + [-1*ones(1,i1-1) zeros(1,i2-i1) -1*ones(1,size(ii,2)-i2-1)];

    ayMesh(kk,:)    = AyV'.*ii(2:end-1);
    BetaMesh(kk,:)  = beta'.*ii(2:end-1) + [50*ones(1,i1-1) zeros(1,i2-i1) 50*ones(1,size(ii,2)-i2-1)];
    MzMesh(kk,:)    = Mz'.*ii(2:end-1);


    
    figure(1)       %'Solution residue'
    hold on
    plot(AyV,TsT,'k')
    plot(AyV,ii(2:end-1),'r')
    grid minor
    
    figure(2)       %Yaw moment diagram
    hold on
    if deltaRef(kk) > 0
        plot(AyV(i1:i2),Mz(i1:i2),'Color',OI.blue,'LineWidth',2)
    elseif deltaRef(kk) < 0
        plot(AyV(i1:i2),Mz(i1:i2),'Color',OI.vermillion,'LineWidth',2)
    else
        plot(AyV(i1:i2),Mz(i1:i2),'Color','k','LineWidth',2)
    end
    kl = 0;
    betaRef =  -7:1:7;
        for kj = 1:length(betaRef)
            xq  = interp1(beta(i1:i2),AyV(i1:i2),betaRef(kj)*pi/180);
            Mzq = interp1(AyV(i1:i2),Mz(i1:i2),xq);
            plot(xq,Mzq,'ro')
            kl = kl+1;
            if isnan(xq)==false
                xqP(kk,kl) = xq;
                MzqP(kk,kl) = Mzq;
            else
                xqP(kk,kl)  = -100;
                MzqP(kk,kl) = -100;
            end
        end

    figure(3)      %'Front tyre normal load (N)'
    hold on
    plot(AyV(i1:i2),Ffz(i1:i2)/forcescale,'k','LineWidth',1.5)
    plot(ay(i1:i2),Ffz(i1:i2)/forcescale,'r','LineWidth',1.5)

    figure(4)      %'Rear tyre normal load (N)'
    hold on
    plot(AyV(i1:i2),Frz(i1:i2)/forcescale,'k','LineWidth',1.5)

    figure(5)       %'Front wheel slip angle (deg)'
    hold on
    plot(AyV(i1:i2),alp_f(i1:i2)*180/pi,'k','LineWidth',1.5)

    figure(6)       %'Rear wheel slip angle (deg)'
    hold on
    plot(AyV(i1:i2),alp_r(i1:i2)*180/pi,'k','LineWidth',1.5)

    figure(7)       %'Front wheel longitudinal slip'
    hold on
    plot(AyV(i1:i2),kf(i1:i2),'k','LineWidth',1.5)

    figure(8)       %'Rear wheel longitudinal slip'
    hold on
    plot(AyV(i1:i2),kr(i1:i2),'k','LineWidth',1.5)

    figure(9)       %'Front wheel side force (N)'
    hold on
    plot(AyV(i1:i2),Ffy(i1:i2)/forcescale,'k','LineWidth',1.5)

    figure(10)       %'Rear wheel side force (N)
    hold on
    plot(AyV(i1:i2),Fry(i1:i2)/forcescale,'k','LineWidth',1.5)

    figure(11)       %'Side slip angle (deg)'
    hold on
    plot(AyV(i1:i2),beta(i1:i2)*180/pi,'k','LineWidth',1.5)

    figure(12)      %Beta diagram
    hold on
    if deltaRef(kk) > 0
        plot(beta(i1:i2)*180/pi,Mz(i1:i2),'Color',OI.blue,'LineWidth',2)
    elseif deltaRef(kk) < 0
        plot(beta(i1:i2)*180/pi,Mz(i1:i2),'Color',OI.vermillion,'LineWidth',2)
    else
        plot(beta(i1:i2)*180/pi,Mz(i1:i2),'Color','k','LineWidth',2)
    end

    Cf          = -DFyAlf(auxdata.tir,Ffz,alp_f,kf); %outputs an instantaneous cornering stiffness (front)
    Cr          = -DFyAlf(auxdata.tir,Frz,alp_r,kr); %outputs an instantaneous cornering stiffness (rear)
    %ConDerv     = a*(Cf+Ffx);
    ConDerv     = a*(Cf*cos(delta) - Cf.*(delta -atan2(omega*a-V*sin(beta),V*cos(beta))).*sin(delta)+Ffx*cos(delta));
    %StabDeriv   = b*Cr - a*Cf;
    StabDeriv   = b*Cr.*(1 - (omega*b - V*sin(beta)).*sin(beta)./V./cos(beta).^2).*(1+(atan2(omega*b-V*sin(beta),V*cos(beta))).^2)...
    - a*Cf.*(1 + (omega*a+V*sin(beta)).*sin(beta)./V./cos(beta).^2).*(1+(atan2(omega*a+V*sin(beta),V*cos(beta))).^2)*cos(delta);

    figure(13)     %'Front tire cornering stiffness'
    hold on
    plot(AyV(i1:i2),Cf(i1:i2),'k','LineWidth',1.5)

    figure(14)   % 'Rear tire cornering stiffness'
    hold on
    plot(AyV(i1:i2),Cr(i1:i2),'k','LineWidth',1.5)

    eta = (b*Cr-a*Cf)*M*g/(a+b);
    figure(15)     %'Understeer coefficient'
    hold on
    plot(AyV(i1:i2),eta(i1:i2),'k','LineWidth',1.5)

    figure(16)      %'Control derivative'
    hold on
    if deltaRef(kk) > 0
        plot(AyV(i1:i2),ConDerv(i1:i2),'r','LineWidth',1.5)
    elseif deltaRef(kk) < 0
        plot(AyV(i1:i2),ConDerv(i1:i2),'b','LineWidth',1.5)
    else
        plot(AyV(i1:i2),ConDerv(i1:i2),'k','LineWidth',1.5)
    end

    figure(17)      %'Stability derivative'
    hold on
    if deltaRef(kk) > 0
        plot(AyV(i1:i2),StabDeriv(i1:i2),'r','LineWidth',1.5)
    elseif deltaRef(kk) < 0
        plot(AyV(i1:i2),StabDeriv(i1:i2),'b','LineWidth',1.5)
    else
        plot(AyV(i1:i2),StabDeriv(i1:i2),'k','LineWidth',1.5)
    end

    YDD = -(a^2.*Cf+b^2.*Cr)/V;
    figure(18)      %yaw damping
    hold on
    if deltaRef(kk) > 0
        plot(AyV(i1:i2),YDD(i1:i2),'r','LineWidth',1.5)
    elseif deltaRef(kk) < 0
        plot(AyV(i1:i2),YDD(i1:i2),'b','LineWidth',1.5)
    else
        plot(AyV(i1:i2),YDD(i1:i2),'k','LineWidth',1.5)
    end
end

%% Plot details

xmin = min(AyVMin);
xmax = max(AyVMax);
ymin = min(MzMin);
ymax = max(MzMax);

figure(1)
box on
grid on
xlabel('Lateral acceleration, $a_y$ (g)')
ylabel('Solution residue')

figure(2)
xlim([xmin xmax])
ylim([ymin-0.1 ymax+0.1])
box on
grid on
xlabel('Normalized lateral acceleration', 'Interpreter', 'latex', 'FontSize', 21)
ylabel('Normalized yaw moment','Interpreter', 'latex', 'FontSize', 21)
set(gca, "TickLabelInterpreter", "latex", "FontSize", 17)
xqP(xqP == -100) = NaN;
MzqP(MzqP == -100) = NaN;
for kj = 1:length(betaRef)
    if betaRef(kj) > 0
        % plot(ay(i1:i2),Mz(i1:i2),'r','LineWidth',1)
        plot(xqP(:,kj),MzqP(:,kj),'ko-.','LineWidth',1,'MarkerFaceColor',OI.blue)
    elseif betaRef(kj) < 0
        plot(xqP(:,kj),MzqP(:,kj),'ko-.','LineWidth',1,'MarkerFaceColor',OI.vermillion)
    else
        plot(xqP(:,kj),MzqP(:,kj),'ko-.','LineWidth',1,'MarkerFaceColor','k')
    end
end
axis equal

figure(3)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Front tyre normal load, $F_z$ (N)','Interpreter', 'latex', 'FontSize', 21);
grid minor
box on

figure(4)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Rear tyre normal load, $F_z$ (N)','Interpreter', 'latex', 'FontSize', 21);
grid minor
box on

figure(5)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Front wheel longitudinal slip, $\kappa_f$','Interpreter', 'latex', 'FontSize', 21);
box on
grid minor

figure(6)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Rear wheel longitudinal slip, $\kappa_r$','Interpreter', 'latex', 'FontSize', 21);
box on
grid minor

figure(7)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Front wheel slip angle, $\alpha_f$ (deg)','Interpreter', 'latex', 'FontSize', 21);
box on
grid minor

figure(8)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Rear wheel slip angle, $\alpha_r$ (deg)','Interpreter', 'latex', 'FontSize', 21);
box on
grid minor

figure(9)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Front wheel side force, $F_y$ (N)','Interpreter', 'latex', 'FontSize', 21);
box on
grid minor

figure(10)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Rear wheel side force, $F_y$ (N)','Interpreter', 'latex', 'FontSize', 21);
box on
grid minor

figure(11)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Side slip angle, $\beta$ (deg)','Interpreter', 'latex', 'FontSize', 21);
box on
grid minor

figure(12)
hold on
xlim([betaRef(1) betaRef(end)]);
xlabel('Side slip angle, $\beta$ (deg)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Normalized yaw moment','Interpreter', 'latex', 'FontSize', 21);
set(gca, "TickLabelInterpreter", "latex", "FontSize", 17)
box on
grid on

figure(13)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Front tyre cornering stiffness, $C_{\alpha f}$','Interpreter', 'latex', 'FontSize', 21);
grid minor
box on

figure(14)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Rear tyre cornering stiffness, $C_{\alpha r}$','Interpreter', 'latex', 'FontSize', 21);
grid minor
box on

figure(15)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Understeer coefficient, $K$','Interpreter', 'latex', 'FontSize', 21);
grid minor
box on

figure(16)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Control derivative, $C_{\delta}$','Interpreter', 'latex', 'FontSize', 21);
box on
grid minor

figure(17)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Stability derivative, $C_{\phi}$','Interpreter', 'latex', 'FontSize', 21);
box on
grid minor

figure(18)
xlim([xmin xmax])
xlabel('Lateral acceleration, $a_y$ (g)','Interpreter', 'latex', 'FontSize', 21);
ylabel('Yaw damping derivative, $C_r$','Interpreter', 'latex', 'FontSize', 21);
box on
grid minor


%timing
elapsedTime = toc;
disp(['Elapsed time: ', num2str(elapsedTime), ' seconds']);
