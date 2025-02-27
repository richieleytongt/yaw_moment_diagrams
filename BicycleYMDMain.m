%% Preamble
tic
clc; clear; close all;

AdigatorRegen = true;
%% Car model input parameters (Ingrid Salisbury Thesis)

%geometry and inertia
w = 3.4;                      %wheelbase (m)
a = 1.6;                      %distance from cg to front axle (m)
b = 1.8;                      %distance from cg to rear axle (m)
h = 0.35;                     %cg height (m)
Af = 2.24;                    %frontal area (m^2)
mass = 720;                   %vehicle mass (kg)
J = 450;                      %Moment of inertia about the z-axis (kg m^2)

%aero data
CD = 0.4;                     %drag coefficient
CS = 0.0;                     %sideforce coefficient
CL = 0.2;                     %lift coefficient (positive down)

%planetary parameters
g = 9.81;                     %acceleration due to gravity (m/s^2)
rho = 1.21094734506627;       %air density (kg/m^3)

%road profile
k   = 0;                      %road inclination (positive uphill)
phi = 0.0*pi/180;             %road banking (negative left NASCAR friendly)

%motion characteristics
V  = 50.0;                    %whole vehicle velocity (m/s)
ax = 0.0;                     %longitudinal acceleration (m/s^2)
s0 = -2;                      %initial negative lateral acceleration guess (g)
sf = 2;                       %initial positive lateral acceleration guess (g)

%Front tire data (Appendix 2, Perantoni and Limebeer, 2014)
Fz1_F           = -2000;                %Reference load 1 (N)
Fz2_F           = -6000;                %Reference load 2 (N)
muxmax1_F       = 1.75;                 %Peak longitudinal friction coefficient at load 1
muxmax2_F       = 1.4;                  %Peak longitudinal friction coefficient at load 2
kmax1_F         = 0.11;                 %Slip coefficient for the friction peak at load 1
kmax2_F         = 0.1;                  %Slip coefficient for the friction peak at load 2
muymax1_F       = 1.8;                  %Peak lateral friction coefficient at load 1
muymax2_F       = 0.45;                 %Peak lateral friction coefficient at load 2
alpmax1_F       = 9;                    %Slip angle for the friction peak at load 1 (deg)
alpmax2_F       = 8;                    %Slip angle for the friction peak at load 2 (deg)
Qx_F            = 1.9;                  %Longitudinal shape factor
Qy_F            = 1.9;                  %Lateral shape factor
Sx_F            = pi./2./atan(Qx_F);
Sy_F            = pi./2./atan(Qy_F);

%Rear tire data (Appendix 2, Perantoni and Limebeer, 2014)
Fz1_R     = -2000;                      %Reference load 1 (N)
Fz2_R     = -6000;                      %Reference load 2 (N)
muxmax1_R = 1.75;                       %Peak longitudinal friction coefficient at load 1
muxmax2_R = 1.4;                        %Peak longitudinal friction coefficient at load 2
kmax1_R   = 0.11;                       %Slip coefficient for the friction peak at load 1
kmax2_R   = 0.1;                        %Slip coefficient for the friction peak at load 2
muymax1_R = 1.8;                        %Peak lateral friction coefficient at load 1
muymax2_R = 0.45;                       %Peak lateral friction coefficient at load 2
alpmax1_R = 9;                          %Slip angle for the friction peak at load 1 (deg)
alpmax2_R = 8;                          %Slip angle for the friction peak at load 2 (deg)
Qx_R      = 1.9;                        %Longitudinal shape factor
Qy_R      = 1.9;                        %Lateral shape factor
Sx_R      = pi./2./atan(Qx_R);
Sy_R      = pi./2./atan(Qy_R);

%to be organized
eps = 1.0e-4;
%% Build 'auxdata' Structure

auxdata.eps 			= eps;

%define the scale factors
lengthscale             = 1/w; 
massscale               = 1/mass;
timescale               = sqrt(g/w);
velocityscale           = lengthscale/timescale; 
accscale                = lengthscale/timescale^2;
forcescale              = massscale*lengthscale/timescale^2;
powerscale              = forcescale*velocityscale;
diffscale               = massscale*lengthscale^2/timescale; 
paramscale              = massscale/timescale^2; 

%store the scale factors
auxdata.lengthscale     = lengthscale;
auxdata.massscale       = massscale;
auxdata.timescale       = timescale;
auxdata.forcescale      = forcescale;
auxdata.paramscale      = paramscale;

%non-dimensionalize and store the geometry and inertia
auxdata.w     = w*lengthscale;
auxdata.a     = a*lengthscale;
auxdata.b     = b*lengthscale;
auxdata.h     = h*lengthscale;
auxdata.Af    = Af*lengthscale^2;
auxdata.M     = mass*massscale;
auxdata.J     = J*lengthscale^2*massscale;

%store aero data
auxdata.CD    = CD;
auxdata.CS    = CS;
auxdata.CL    = CL;

%non-dimensionalize and store planetary parameters
auxdata.g      = g*accscale;
auxdata.rho    = rho*massscale/lengthscale^3;

%store road profile
auxdata.k      = k;
auxdata.phi    = phi;

%non-dimensionalize and store motion characteristics
auxdata.V      = V*velocityscale;
auxdata.ax     = ax*accscale;
auxdata.s0     = s0;
auxdata.sf     = sf;

%non-dimensionalize and store front tire data
auxdata.Fz1_F     = Fz1_F*forcescale;
auxdata.Fz2_F     = Fz2_F*forcescale;
auxdata.muxmax1_F = muxmax1_F;
auxdata.muxmax2_F = muxmax2_F;
auxdata.kmax1_F   = kmax1_F;
auxdata.kmax2_F   = kmax2_F;
auxdata.muymax1_F = muymax1_F;
auxdata.muymax2_F = muymax2_F;
auxdata.alpmax1_F = alpmax1_F*pi/180;
auxdata.alpmax2_F = alpmax2_F*pi/180;
auxdata.Qx_F      = Qx_F;
auxdata.Qy_F      = Qy_F;
auxdata.Sx_F      = Sx_F;
auxdata.Sy_F      = Sy_F;

%non-dimensionalize and store rear tire data
auxdata.Fz1_R     = Fz1_R*forcescale;
auxdata.Fz2_R     = Fz2_R*forcescale;
auxdata.muxmax1_R = muxmax1_R;
auxdata.muxmax2_R = muxmax2_R;
auxdata.kmax1_R   = kmax1_R;
auxdata.kmax2_R   = kmax2_R;
auxdata.muymax1_R = muymax1_R;
auxdata.muymax2_R = muymax2_R;
auxdata.alpmax1_R = alpmax1_R*pi/180;
auxdata.alpmax2_R = alpmax2_R*pi/180;
auxdata.Qx_R      = Qx_R;
auxdata.Qy_R      = Qy_R;
auxdata.Sx_R      = Sx_R;
auxdata.Sy_R      = Sy_R;

%withdraw nondimensional terms
M   = auxdata.M;                     %withdraw nondimensional mass from auxdata structure
g   = auxdata.g;                     %withdraw nondimensional gravity from auxdata structure
rho = auxdata.rho;                   %withdraw nondimensional air density from auxdata structure
Af  = auxdata.Af;                    %withdraw nondimensional frontal area from auxdata structure
h   = auxdata.h;                     %withdraw nondimensional cg height from auxdata structure
a   = auxdata.a;                     %withdraw nondimensional a
b   = auxdata.b;                     %withdraw nondimensional b
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
setup.functions.continuous           = @BicycleMMMContinuous;   %name of the continuous function
setup.functions.endpoint             = @BicycleMMMEndpoint;     %name of the endpoint function
setup.nlp.solver                     = 'ipopt'; % 'ipopt'       %default solver: gradient descent algorithm
setup.nlp.ipoptoptions.linear_solver = 'ma57';                  
setup.nlp.ipoptoptions.maxiterations = 2000;                    %default value
setup.method                         = 'RPM-Differentiation';   %default method
setup.bounds                         = bounds;                  %deposit bounds structure into setup structure
setup.guess                          = guess;                   %deposit guess structure into setup structure
setup.derivatives.supplier           = 'adigator';              
if AdigatorRegen == false
    setup.adigatorgrd.continuous     = @BicycleMMMContinuousADiGatorGrd;
    setup.adigatorgrd.endpoint       = @BicycleMMMEndpointADiGatorGrd;
    setup.adigatorhes.continuous     = @BicycleMMMContinuousADiGatorHes;
    setup.adigatorhes.endpoint       = @BicycleMMMEndpointADiGatorHes;
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

    %withdraw front tire data
    Fz1_F     = auxdata.Fz1_F;
    Fz2_F     = auxdata.Fz2_F;
    muxmax1_F = auxdata.muxmax1_F;
    muxmax2_F = auxdata.muxmax2_F;
    kmax1_F   = auxdata.kmax1_F;
    kmax2_F   = auxdata.kmax2_F;
    muymax1_F = auxdata.muymax1_F;
    muymax2_F = auxdata.muymax2_F;
    alpmax1_F = auxdata.alpmax1_F;
    alpmax2_F = auxdata.alpmax2_F;
    Qx_F      = auxdata.Qx_F;
    Qy_F      = auxdata.Qy_F;
    Sx_F      = auxdata.Sx_F;
    Sy_F      = auxdata.Sy_F;

    %store front tire data into a new structure
    dataF.Fz1     = auxdata.Fz1_F;
    dataF.Fz2     = auxdata.Fz2_F;
    dataF.muxmax1 = auxdata.muxmax1_F;
    dataF.muxmax2 = auxdata.muxmax2_F;
    dataF.kmax1   = auxdata.kmax1_F;
    dataF.kmax2   = auxdata.kmax2_F;
    dataF.muymax1 = auxdata.muymax1_F;
    dataF.muymax2 = auxdata.muymax2_F;
    dataF.alpmax1 = auxdata.alpmax1_F;
    dataF.alpmax2 = auxdata.alpmax2_F;
    dataF.Qx      = auxdata.Qx_F;
    dataF.Qy      = auxdata.Qy_F;
    dataF.Sx      = auxdata.Sx_F;
    dataF.Sy      = auxdata.Sy_F;

    %front tire force calculation (Appendix 1, Perantoni and Limebeer, 2014)
    muxmaxf = (Ffz - Fz1_F).*(muxmax2_F-muxmax1_F)./(Fz2_F-Fz1_F) + muxmax1_F;
    muymaxf = (Ffz - Fz1_F).*(muymax2_F-muymax1_F)./(Fz2_F-Fz1_F) + muymax1_F;
    kmaxf   = (Ffz - Fz1_F).*(kmax2_F-kmax1_F)./(Fz2_F-Fz1_F)+kmax1_F;
    alpmaxf = (Ffz - Fz1_F).*(alpmax2_F-alpmax1_F)./(Fz2_F-Fz1_F)+alpmax1_F;
    knf     = kf./kmaxf;
    alpnf   = alp_f./alpmaxf;
    rhof    = sqrt(knf.^2+alpnf.^2);
    muxf    = muxmaxf.*sin(Qx_F.*atan(Sx_F.*rhof));
    muyf    = muymaxf.*sin(Qy_F.*atan(Sy_F.*rhof));
    Ffx     = muxf.*Ffz.*knf./rhof;
    Ffy     = muyf.*Ffz.*alpnf./rhof;

    %withdraw rear tire data
    Fz1_R     = auxdata.Fz1_R;
    Fz2_R     = auxdata.Fz2_R;
    muxmax1_R = auxdata.muxmax1_R;
    muxmax2_R = auxdata.muxmax2_R;
    kmax1_R   = auxdata.kmax1_R;
    kmax2_R   = auxdata.kmax2_R;
    muymax1_R = auxdata.muymax1_R;
    muymax2_R = auxdata.muymax2_R;
    alpmax1_R = auxdata.alpmax1_R;
    alpmax2_R = auxdata.alpmax2_R;
    Qx_R      = auxdata.Qx_R;
    Qy_R      = auxdata.Qy_R;
    Sx_R      = auxdata.Sx_R;
    Sy_R      = auxdata.Sy_R;

    %store rear tire data into a new structure
    dataR.Fz1     = auxdata.Fz1_R;
    dataR.Fz2     = auxdata.Fz2_R;
    dataR.muxmax1 = auxdata.muxmax1_R;
    dataR.muxmax2 = auxdata.muxmax2_R;
    dataR.kmax1   = auxdata.kmax1_R;
    dataR.kmax2   = auxdata.kmax2_R;
    dataR.muymax1 = auxdata.muymax1_R;
    dataR.muymax2 = auxdata.muymax2_R;
    dataR.alpmax1 = auxdata.alpmax1_R;
    dataR.alpmax2 = auxdata.alpmax2_R;
    dataR.Qx      = auxdata.Qx_R;
    dataR.Qy      = auxdata.Qy_R;
    dataR.Sx      = auxdata.Sx_R;
    dataR.Sy      = auxdata.Sy_R;

    %rear tyre force calculation (Appendix 1, Perantoni and Limebeer, 2014)
    muxmaxr = (Frz - Fz1_R).*(muxmax2_R-muxmax1_R)./(Fz2_R-Fz1_R) + muxmax1_R;
    muymaxr = (Frz - Fz1_R).*(muymax2_R-muymax1_R)./(Fz2_R-Fz1_R) + muymax1_R;
    kmaxr   = (Frz - Fz1_R).*(kmax2_R-kmax1_R)./(Fz2_R-Fz1_R)+kmax1_R;
    alpmaxr = (Frz - Fz1_R).*(alpmax2_R-alpmax1_R)./(Fz2_R-Fz1_R)+alpmax1_R;
    knr     = kr./kmaxr;
    alpnr   = alp_r./alpmaxr;
    rhor    = sqrt(knr.^2+alpnr.^2);
    muxr    = muxmaxr.*sin(Qx_R.*atan(Sx_R.*rhor));
    muyr    = muymaxr.*sin(Qy_R.*atan(Sy_R.*rhor));
    Frx     = muxr.*Frz.*knr./rhor;
    Fry     = muyr.*Frz.*alpnr./rhor;

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
    ii = [0, TsT' < 0.005,0];
    i1 = min(strfind(ii,[0 1]));
    i2 = max(strfind(ii,[1 0]));
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
        plot(AyV(i1:i2),Mz(i1:i2),'r','LineWidth',1)
    elseif deltaRef(kk) < 0
        plot(AyV(i1:i2),Mz(i1:i2),'b','LineWidth',1)
    else
        plot(AyV(i1:i2),Mz(i1:i2),'k','LineWidth',2)
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
        plot(beta(i1:i2)*180/pi,Mz(i1:i2),'r','LineWidth',1.5)
    elseif deltaRef(kk) < 0
        plot(beta(i1:i2)*180/pi,Mz(i1:i2),'b','LineWidth',1.5)
    else
        plot(beta(i1:i2)*180/pi,Mz(i1:i2),'k','LineWidth',1.5)
    end

    Cf          = -DFyAlf(dataF,Ffz,alp_f,kf); %outputs an instantaneous cornering stiffness (front)
    Cr          = -DFyAlf(dataR,Frz,alp_r,kr); %outputs an instantaneous cornering stiffness (rear)
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
xlabel('Lateral acceleration (G)')
ylabel('Solution residue')

figure(2)
xlim([xmin xmax])
ylim([ymin-0.1 ymax+0.1])
box on
grid on
xlabel('Normalised lateral acceleration')
ylabel('Normalized yaw moment')
xqP(xqP == -100) = NaN;
MzqP(MzqP == -100) = NaN;
for kj = 1:length(betaRef)
    if betaRef(kj) > 0
        % plot(ay(i1:i2),Mz(i1:i2),'r','LineWidth',1)
        plot(xqP(:,kj),MzqP(:,kj),'ko-.','LineWidth',1,'MarkerFaceColor','r')
    elseif betaRef(kj) < 0
        plot(xqP(:,kj),MzqP(:,kj),'ko-.','LineWidth',1,'MarkerFaceColor','b')
    else
        plot(xqP(:,kj),MzqP(:,kj),'ko-.','LineWidth',1,'MarkerFaceColor','k')
    end
end

figure(3)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Front tyre normal load (N)');
grid minor
box on

figure(4)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Rear tyre normal load (N)');
grid minor
box on

figure(5)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Front wheel longitudinal slip');
box on
grid minor

figure(6)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Rear wheel longitudinal slip');
box on
grid minor

figure(7)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Front wheel slip angle (deg)');
box on
grid minor

figure(8)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Rear wheel slip angle (deg)');
box on
grid minor

figure(9)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Front wheel side force (N)');
box on
grid minor

figure(10)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Rear wheel side force (N)');
box on
grid minor

figure(11)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Side slip angle (deg)');
box on
grid minor

figure(12)
hold on
% xlim([beta(i1)*180/pi beta(i2)*180/pi]);
xlim([betaRef(1) betaRef(end)]);
xlabel('Side slip angle (deg)');
ylabel('Normaled yaw moment');
box on
grid minor

figure(13)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Front tyre cornering stiffness');
grid minor
box on

figure(14)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Rear tyre cornering stiffness');
grid minor
box on

figure(15)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Understeer coefficient');
grid minor
box on

figure(16)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Control derivative');
box on
grid minor

figure(17)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Stability derivative');
box on
grid minor

figure(18)
xlim([xmin xmax])
xlabel('Lateral acceleration (G)');
ylabel('Yaw damping derivative');
box on
grid minor


%timing
elapsedTime = toc;
disp(['Elapsed time: ', num2str(elapsedTime), ' seconds']);
