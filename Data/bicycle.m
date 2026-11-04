%% Car model input parameters (Ingrid Salisbury Thesis)

%geometry and inertia
w = 3.4;                      %wheelbase (m)
a = 1.6;                      %distance from cg to front axle (m)
b = 1.8;                      %distance from cg to rear axle (m)
h = 0.35;                     %cg height (m)
Af = 2.24;                    %frontal area (m^2)
M = 720;                   %vehicle mass (kg)
J = 450;                      %Moment of inertia about the z-axis (kg m^2)

%aero data
CD = 0.4;                     %drag coefficient
CS = 0.0;                     %sideforce coefficient
CL = 0.2;                     %lift coefficient (positive down)

%send scaled information to auxdata
auxdata.w     = w*lengthscale;
auxdata.a     = a*lengthscale;
auxdata.b     = b*lengthscale;
auxdata.h     = h*lengthscale;
auxdata.Af    = Af*lengthscale^2;
auxdata.M     = M*massscale;
auxdata.J     = J*massscale*lengthscale^2;
auxdata.CD    = CD;
auxdata.CS    = CS;
auxdata.CL    = CL;
