trial = 12;
%%
trial=trial+1;
dt = 0.003;
T = 30.0;
ttape = 0:dt:T;
N = numel(ttape);
A = floor(N/3);
B = N-2*A;
ydottape = ... %[0.00*ones(1,A) 0.00*ones(1,B) -0.00*ones(1,A)];
    [0*ttape(1:7.5/dt) 0.08*cos((2*pi/1.0)*ttape(7.5/dt+1:end))]; %+0.03*sin(10.1*ttape);
dlmwrite(sprintf('sysid%d.dat',trial),[numel(ydottape) ydottape]','\n');
%%
!observer_controller.exe 2 sysid.dat
%%
out = dlmread(sprintf('out_sysid%d.dat',trial));
yout = out(1:2:end-1);
thetaout = out(2:2:end-1);
plot(ttape,thetaout);