out = dlmread('out_sysid0.dat');
dt=.003;

df = [0.003059  	 0.035187  	 0.118739  	 0.143928  	 0.000000  	 -0.143928  	 -0.118739  	 -0.035187  	 -0.003059];
df=df/dt;

y=out(1:3:end-1);
theta=out(2:3:end-1);
td_obs=out(3:3:end-1);

t=[0:dt:dt*(length(y)-1)];

plot(t, filter(df, 1, theta), t, td_obs)