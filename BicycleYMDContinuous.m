function phaseout = BicycleYMDContinuous(input)
%input is a structure that is defined behind closed doors. We are told that
%it contains the following sub-structures: 'phase' and 'auxdata'

ay              = input.phase(1).time;
x               = input.phase(1).state;
U               = input.phase(1).control;

[dynamics,path,R1,R2] = BicycleYMDDAE(ay,x,U,input.auxdata);

phaseout.dynamics       = dynamics;
phaseout.path           = path;
phaseout.integrand(:,1) = R1.^2 + R2.^2 + 1.0e-2.*(U(:,1).^2 + U(:,2).^2 + U(:,3).^2); %(Eq: 14)



end
