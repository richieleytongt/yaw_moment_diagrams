
function dyFalf = DFyAlf(data,Fz,alf,k)

%inputs are (structure, column vector, column vector, column vector)
%tire data, downforce, slip angle, slip ratio
%Linearize the lateral force characteristic around (Fz,alf,k)

Fz1     = data.Fz1;
Fz2     = data.Fz2;
muxmax1 = data.muxmax1;
muxmax2 = data.muxmax2;
kmax1   = data.kmax1;
kmax2   = data.kmax2;
muymax1 = data.muymax1;
muymax2 = data.muymax2;
alfmax1 = data.alpmax1;
alfmax2 = data.alpmax2;
Qx      = data.Qx;
Qy      = data.Qy;
Sx      = data.Sx;
Sy      = data.Sy;

%all coming from Maple. No idea what's going on here.
%try to do this in matlab, symbolically.
t2      = 0.1e1./(Fz2 - Fz1);
t5      = Fz - Fz1;
t8      = (t5.*(kmax2 - kmax1)*t2 + kmax1).^2;
t10     = k.^2;
t15     = t5.*(alfmax2 - alfmax1)*t2 + alfmax1;
t16     = t15.^2;
t18     = alf.^2;
t20     = t10./t8 + t18./t16;
t21     = sqrt(t20);
t25     = 0.1e1./t16./t15;
t28     = atan(Qy);
t29     = 0.1e1./t28;
t33     = atan(pi.*t29*t21./0.2e1);
t34     = Qy.*t33;
t35     = sin(t34);
t40     = t5.*(muymax2 - muymax1)*t2 + muymax1;
t41     = t40.*t35.*Fz;
t52     = t28.^2;
t55     = pi.^2;
t61     = cos(t34);

%equilibrium cornering stiffness
dyFalf  = -t41.*t18.*t25./t21./t20 + t41./t15./t21 + t40.* t61.* Qy./(t55./t52.*t20./0.4e1 + 0.1e1).* pi.*t29.*Fz.*t18.*t25./t20./0.2e1; 
end