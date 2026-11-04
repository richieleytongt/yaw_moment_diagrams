function [dynamics,path,R1,R2] = BicycleYMDDAE(ay,x,U,data)

%% Withdrawing inputs
%car model parameters
eps             = data.eps;

%withdraw geometry and inertia parameters from data
a               = data.a;
b               = data.b;
h               = data.h;
Af              = data.Af;
M               = data.M;

%withdraw aero data
CD              = data.CD;
CS              = data.CS;
CL              = data.CL;

%withdraw planetary parameters from data
g               = data.g;
rho             = data.rho;

%withdraw road profile from data
k               = data.k;
phi 			= data.phi;

%withdraw motion characteristics from data
V               = data.V;
ax              = data.ax;
delta           = data.delta;

% The States
beta       		= x(:,1);
k_f             = x(:,2);
k_r             = x(:,3);

% The Controls
dbeta       = U(:,1);
dk_f        = U(:,2);
dk_r        = U(:,3);
Ffz         = U(:,4);
Frz         = U(:,5);

%% Calculations

% Project velocities
u           = V.*cos(beta); %(Eq: 19)
v           = V.*sin(beta); %(Eq: 19)
R           = (V.^2 + sqrt(V.^4 - 4*k^2*ay.^2))./(2*cos(phi).*sqrt(ay.^2 + eps)); %rho in the paper (Eq: 27)
omega       = -sqrt(sqrt(ay.^2 + eps)./R./cos(phi)).*2/pi.*atan(1000*(ay+eps)); %(Eq: 27)
omega_Bz    = omega.*R.*cos(phi).^2./sqrt(R.^2.*cos(phi).^2 + k.^2); %(Eq: 35)

% rear and front slip angles
alp_f   = atan2(v+omega_Bz.*a,u) - delta;   %(Eq: 41)
alp_r   = atan2(v-omega_Bz.*b,u);           %(Eq: 40)

% Compute aerodynamic forces and moments
Fax 	= 0.5*rho*Af*CD.*u.^2;    %(Eq: 46)
Fay 	= 0.5*rho*Af*CS.*u.^2;    %(Eq: 47)
Faz 	= 0.5*rho*Af*CL.*u.^2;    %(Eq: 48)

%Tire forces
[Ffx, Ffy] = Tire_force(alp_f, k_f, Ffz, data.tir);
[Frx, Fry] = Tire_force(alp_r, k_r, Frz, data.tir);

% Gravitational accelerations (Eq: 36)
Gx      = -g./sqrt(cos(phi)^2.*R.^2+k^2).*(R.*sin(beta).*sin(2*phi)/2 - k*cos(beta));
Gy      = g./sqrt(cos(phi)^2.*R.^2+k^2).*(R.*cos(beta).*sin(2*phi)/2 - k*sin(beta));
Gz      = g./sqrt(cos(phi)^2.*R.^2+k^2).*(R.*cos(phi)^2);

% Lateral accelerations (Eq: 37)
AxB     =   ay.*cos(phi).*sin(beta);
AyB     =   -ay.*cos(phi).*cos(beta);
AzB     =   ay.*sin(phi);

% Force balance equations
Fx 		= cos(delta).*Ffx - sin(delta).*Ffy + Frx + M*Gx - Fax;      %(Eq: 44)
Fy 		= cos(delta).*Ffy + sin(delta).*Ffx + Fry + M*Gy + Fay;      %(Eq: 45)
Fz 		= Frz + Ffz + M*Gz + Faz + AzB; %[vertical force constraint]  (Eq: 49)

% Pitch moment
My 		= a.*Ffz - b.*Frz - h.*Fx; %[pitch moment constraint] (Eq: 50)


%% Outputs

dynamics   = [dbeta,dk_f,dk_r];            %derivative of state vector
path       = [Fz,My,alp_r,k_r,alp_f,k_f];  %constraint vector
R1         = ax.*cos(beta) - AxB - Fx./M;  %(Eq: 42)
R2 	       = ax.*sin(beta) - AyB - Fy./M;  %(Eq: 43)


end
