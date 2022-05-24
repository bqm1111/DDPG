function output = createDelay(state,N,dt)
output = repmat(state,N,1);
for i = N:-1:2
    output(i-1,1) =  output(i,1) + output(i,2)*dt;
    output(i-1,5) =  output(i,5) + output(i,6)*dt;
end
end