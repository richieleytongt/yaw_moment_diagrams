function [dynamics,path,R1,R2] = BicycleMMMDAE(ay,x,U,data)
%[dynamics,path,R1,R2] = BicycleMMMDAE(ay,x,U,data)

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

%withdraw front tire parameters from data
Fz1_F          = data.Fz1_F;
Fz2_F          = data.Fz2_F; 
muxmax1_F      = data.muxmax1_F;
muxmax2_F      = data.muxmax2_F;
kmax1_F        = data.kmax1_F;
kmax2_F        = data.kmax2_F;
muymax1_F      = data.muymax1_F;
muymax2_F      = data.muymax2_F;
alpmax1_F      = data.alpmax1_F;
alpmax2_F      = data.alpmax2_F;
Qx_F           = data.Qx_F;
Qy_F           = data.Qy_F;
Sx_F           = data.Sx_F;
Sy_F           = data.Sy_F;

%withdraw rear tire parameters from data
Fz1_R          = data.Fz1_R;
Fz2_R          = data.Fz2_R; 
muxmax1_R      = data.muxmax1_R;
muxmax2_R      = data.muxmax2_R;
kmax1_R        = data.kmax1_R;
kmax2_R        = data.kmax2_R;
muymax1_R      = data.muymax1_R;
muymax2_R      = data.muymax2_R;
alpmax1_R      = data.alpmax1_R;
alpmax2_R      = data.alpmax2_R;
Qx_R           = data.Qx_R;
Qy_R           = data.Qy_R;
Sx_R           = data.Sx_R;
Sy_R           = data.Sy_R;

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

% front right tyre force calculation
muxmaxf = (Ffz - Fz1_F).*(muxmax2_F-muxmax1_F)./(Fz2_F-Fz1_F) + muxmax1_F;
muymaxf = (Ffz - Fz1_F).*(muymax2_F-muymax1_F)./(Fz2_F-Fz1_F) + muymax1_F;
kmaxf   = (Ffz - Fz1_F).*(kmax2_F-kmax1_F)./(Fz2_F-Fz1_F)+kmax1_F;
alpmaxf = (Ffz - Fz1_F).*(alpmax2_F-alpmax1_F)./(Fz2_F-Fz1_F)+alpmax1_F;
knf     = k_f./kmaxf;
alpnf   = alp_f./alpmaxf;
rhof    = sqrt(knf.^2+alpnf.^2);
muxf    = muxmaxf.*sin(Qx_F.*atan(Sx_F.*rhof));
muyf    = muymaxf.*sin(Qy_F.*atan(Sy_F.*rhof));
Ffx     = muxf.*Ffz.*knf./rhof;
Ffy     = muyf.*Ffz.*alpnf./rhof;

% rear right tyre force calculation
muxmaxr = (Frz - Fz1_R).*(muxmax2_R-muxmax1_R)./(Fz2_R-Fz1_R) + muxmax1_R;
muymaxr = (Frz - Fz1_R).*(muymax2_R-muymax1_R)./(Fz2_R-Fz1_R) + muymax1_R;
kmaxr   = (Frz - Fz1_R).*(kmax2_R-kmax1_R)./(Fz2_R-Fz1_R)+kmax1_R;
alpmaxr = (Frz - Fz1_R).*(alpmax2_R-alpmax1_R)./(Fz2_R-Fz1_R)+alpmax1_R;
knr     = k_r./kmaxr;
alpnr   = alp_r./alpmaxr;
rhor    = sqrt(knr.^2+alpnr.^2);
muxr    = muxmaxr.*sin(Qx_R.*atan(Sx_R.*rhor));
muyr    = muymaxr.*sin(Qy_R.*atan(Sy_R.*rhor));
Frx     = muxr.*Frz.*knr./rhor;
Fry     = muyr.*Frz.*alpnr./rhor;

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
