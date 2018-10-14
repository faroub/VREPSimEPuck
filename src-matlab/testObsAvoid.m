
% for one repeller
beta_1=20;
beta_2=1;
dist_obs=zeros(1,100);
lambda_obs=zeros(1,100);
rob_rad=0.053/2;

dist_obs(1:20)=0.04;
offset=0;

for i=21:100
    
    dist_obs(i)=0.04-offset;
    offset=offset+0.0005;
end

for i=1:100
    lambda_obs(i) =  beta_1 * exp( -beta_2*dist_obs(i)/ rob_rad );
end
hold on
plot(dist_obs,lambda_obs)
