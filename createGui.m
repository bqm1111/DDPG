x1 = -pi:pi/100:pi;
x2 = -pi:pi/100:pi;
%Generate a state list
states=zeros(length(x1)*length(x2),2); % 2 Column matrix of all possible combinations of the discretized state.
index=1;
for j=1:length(x1)
    for k = 1:length(x2)
        states(index,1)=x1(j);
        states(index,2)=x2(k);
        index=index+1;
    end
end
for j=1:length(x1)
    for k = 1:length(x2)
        for i = length(actions)
            train_states(index,1) = x1(j);
            train_states(index,2) = x2(k);
            train_states(index,3) = actions(i);
            train_R(index,1)      = rewardFunc(x1(j),x2(k));
            index=index+1;
        end
    end
end
R = rewardFunc(states(:,1),states(:,2));
Q = repmat(R,[1,length(actions)]);
V = zeros(size(states,1),1);
Vorig = reshape(max(Q,[],2),[length(x2),length(x1)]);

%% Set up the pendulum plot
panel = figure;
% panel.Position = [680 558 1000 400];
panel.Color = [1 1 1];
subplot(1,4,1)
hold on
% Axis for the pendulum animation
f = plot(0,0,'b','LineWidth',10); % Pendulum stick
axPend = f.Parent;
axPend.XTick = []; % No axis stuff to see
axPend.YTick = [];
axPend.Visible = 'off';
axPend.Position = [0.01 0.5 0.3 0.3];
axPend.Clipping = 'off';
axis equal
axis([-1.2679 1.2679 -1 1]);
plot(0.001,0,'.k','MarkerSize',50); % Pendulum axis point
hold off

%% Set up the state-value map plot (displays the value of the best action at every point)
colormap('hot');
subplot(1,4,[2:4]);
hold on;
grid on;
map = imagesc(reshape(R,[length(x2),length(x1)]));
axMap = map.Parent;
axMap.XTickLabels = {'-pi' '-pi/2' '0' 'pi/2' 'pi'};
axMap.XTick = [1 floor(length(x1)/4)+1 floor(length(x1)/2)+1 floor(length(x1)*3/4)+1 length(x1)];
axMap.YTickLabels = {'-pi' '-pi/2' '0' 'pi/2' 'pi'};
axMap.YTick = [1 floor(length(x2)/4)+1 floor(length(x2)/2)+1 floor(length(x2)*3/4)+1 length(x2)];
axMap.XLabel.String = 'Angle (rad)';
axMap.YLabel.String = 'Angular rate (rad/s)';
axMap.Visible = 'on';
axMap.Color = [0.6 0.6 0.5];
axMap.XLim = [1 length(x1)];
axMap.YLim = [1 length(x2)];
axMap.Box = 'off';
axMap.FontSize = 10;
caxis([3*min(R),max(R)])
pathmap = plot(NaN,NaN,'.g','MarkerSize',30); % The green marker that travels through the state map to match the pendulum animation
map.CData = V;
hold off