% Helical Surface Plots
clc, clear, close all

% Okabe-Ito color palette
OI.orange     = [230, 159, 0]   / 255;
OI.skyBlue    = [86, 180, 233]  / 255;
OI.blueGreen  = [0, 158, 115]   / 255;
OI.yellow     = [240, 228, 66]  / 255;
OI.blue       = [0, 114, 178]   / 255;
OI.vermillion = [213, 94, 0]    / 255;
OI.purple     = [204, 121, 167] / 255;

% Nord color palette
Nord.nord0  = [46, 52, 64]   / 255; 
Nord.nord1  = [59, 66, 82]   / 255;
Nord.nord2  = [67, 76, 94]   / 255;
Nord.nord3  = [76, 86, 106]  / 255;
Nord.nord4  = [216, 222, 233]/ 255;
Nord.nord5  = [229, 233, 240]/ 255;
Nord.nord6  = [236, 239, 244]/ 255;
Nord.nord7  = [143, 188, 187]/ 255;
Nord.nord8  = [136, 192, 208]/ 255;
Nord.nord9  = [129, 161, 193]/ 255;
Nord.nord10 = [94, 129, 172] / 255;

%% Parametric representation of an oblique helicoid

mu      = 5*pi/180; % uphill angle
phi     = 21*pi/180; % camber angle
funx    = @(u,v) u.*cos(v).*cos(phi);
funy    = @(u,v) -(u.*sin(v).*cos(phi));
funz    = @(u,v) -(sin(mu)*v + u*sin(phi));

figure(1);
fsurf(funx,funy,funz,[0.5 1 0 2.5*pi],'EdgeColor',Nord.nord0,'FaceColor',Nord.nord4,'LineWidth',2,'MeshDensity',13)
hold on
fsurf(funx,funy,funz,[.74 .76 0 2.5*pi],'EdgeColor',Nord.nord8,'FaceColor',Nord.nord8,'LineWidth',3,'MeshDensity',1)
% plot3([0 0],[0 0],[0 1.2],'k','LineWidth',2)
% plot3(0,0,0,'kp','LineWidth',2)
% plot3(0,0,1.2,'kp','LineWidth',2)
% title('oblique helicoid')
xlabel('Inertial X','Interpreter','latex','FontSize',21)
ylabel('Inertial Y','Interpreter','latex','FontSize',21)
zlabel('Inertial Z','Interpreter','latex','FontSize',21)
set(gca,'ticklabelinterpreter','latex','FontSize',17)
set(gca,'YDir','reverse')
set(gca,'ZDir','reverse')
axis equal

% %% Parametric representation of a cone

% mu      = 0*pi/180; % uphill angle
% phi     = 20*pi/180; % camber angle
% funx    = @(u,v) u.*cos(v).*cos(phi);
% funy    = @(u,v) u.*sin(v).*cos(phi);
% funz    = @(u,v) sin(mu)*v + u*sin(phi);

% figure(2);
% fsurf(funx,funy,funz,[0.5 1 0 2*pi],'EdgeColor','k','FaceColor','none','LineWidth',1.5,'MeshDensity',21)
% hold on
% fsurf(funx,funy,funz,[.74 .76 0 2*pi],'EdgeColor','r','FaceColor','none','LineWidth',1,'MeshDensity',21)
% plot3([0 0],[0 0],[0 0.5],'r','LineWidth',2)
% plot3(0,0,0,'rp','LineWidth',2)
% plot3(0,0,0.5,'rp','LineWidth',2)
% title('conical surface')
% axis equal

% %% Parametric representation of a right helicoid

% mu      = 5*pi/180; % uphill angle
% phi     = 0*pi/180; % camber angle
% funx    = @(u,v) u.*cos(v).*cos(phi);
% funy    = @(u,v) u.*sin(v).*cos(phi);
% funz    = @(u,v) sin(mu)*v + u*sin(phi);

% figure(3);
% fsurf(funx,funy,funz,[0.5 1 0 2.5*pi],'EdgeColor','k','FaceColor','none','LineWidth',1.5,'MeshDensity',20)
% hold on
% fsurf(funx,funy,funz,[.74 .76 0 2.5*pi],'EdgeColor','r','FaceColor','none','LineWidth',1,'MeshDensity',20)
% plot3([0 0],[0 0],[0 1.0],'r','LineWidth',2)
% plot3(0,0,0,'rp','LineWidth',2)
% plot3(0,0,1.0,'rp','LineWidth',2)
% title('Right helicoid')
% axis equal

% %% Parametric representation of a toroidal ring

% mu      = 0*pi/180; % uphill angle
% phi     = 0*pi/180; % camber angle
% funx    = @(u,v) u.*cos(v).*cos(phi);
% funy    = @(u,v) u.*sin(v).*cos(phi);
% funz    = @(u,v) sin(mu)*v + u*sin(phi);

% figure(4);
% fsurf(funx,funy,funz,[0.5 1 0 2*pi],'EdgeColor','k','FaceColor','none','LineWidth',1.5,'MeshDensity',21)
% hold on
% fsurf(funx,funy,funz,[.74 .76 0 2*pi],'EdgeColor','r','FaceColor','none','LineWidth',1,'MeshDensity',21)
% plot3([0 0],[0 0],[0 1.0],'r','LineWidth',2)
% plot3(0,0,0,'rp','LineWidth',2)
% plot3(0,0,1.0,'rp','LineWidth',2)
% title('Toroidal ring')
% axis equal