% clear all;
% close all;
rewardFunc = @(x,xdot)(-0.1*(x*eye(2)*x')/100 + -1*(xdot*eye(2)*xdot')/100); 
%% Create environment
environment.substeps = 15;        % Time calculator in step
environment.delay    = 1;        % State delay step
environment.dt       = 0.001;     % Sample time
% Delay 01 steps: Reward = -700;  Learning Rate: 0.00002;
% Delay 05 steps: Reward = -700;  Learning Rate: 0.00002;
% Delay 10 steps: Reward = -700;  Learning Rate: 0.00001;
% Delay 15 steps: Reward = -2500; Learning Rate: 0.0004;

train.totalSteps   =  0;
train.stopReward   = -500;
train.done         = 0.01;
train.learningRate =  0.00002;
train.gain         = 20;
train.gainKp = 40;
train.gainKi = 30;
train.gainKd = 5;
%% Parameter training Q-learing
parameter.sigma      = [30 20 5]; 
parameter.noiseDecay = 0.99;
parameter.discount   = 0.99; 
parameter.tau        = 1;   
parameter.maxEpi     = 1000; 
parameter.maxit      = 500; 
%% Create Memory
memory.capacity      = 20000;
memory.batch_size    = 128;
memory.index         = 0;
memory.data          = zeros(40,memory.capacity);
memory.train.data    = zeros(40,memory.batch_size);
memory.train.index   = zeros(1,memory.batch_size);

%% Create Critic and Actor Net
nnQ   = Network( [18,32,64,1], {'ReLU','ReLU'} );
nnKu  = Network( [6,8,3], {'ReLU'} );
nnKw  = Network( [6,8,3], {'ReLU'} );
nnKu.set_output('Sig');
nnKw.set_output('Sig');

nnQt  = nnQ;
nnKut = nnKu;
nnKwt = nnKw;

optimQ  = ADAM(length(nnQ.W));
optimQ.alpha  = train.gain * train.learningRate;          % Learning rate Q-net = 0.0005;
optimKu = ADAM(length(nnKu.W));
optimKu.alpha  = train.learningRate;          % Learning rate A-net = 0.0003;
optimKw = ADAM(length(nnKw.W));
optimKw.alpha = train.learningRate;          % Learning rate A-net = 0.0003;
%% Create figure for train
h = figure(1);
h.Position = [685 42 681 642];
hPosition = h.Position;
subplot(211);
h1 = animatedline('Color','r','Marker','o');
h2 = animatedline('Color','b','Marker','*');
legend('EpisodeReward','AverageReward','Location','northwest');
xlabel('Episodes'); ylabel('Reward');grid on;

subplot(212);
h3 = animatedline('Color','g','Marker','x');
legend('Loss','Location','northwest');
xlabel('Episodes'); ylabel('Loss');grid on;

%% Start learning!
for episodes = 1:parameter.maxEpi
    parameterReset;
%     state0_o = Delay(end,:);
    train.EpisodeReward(episodes) = 0;
    train.AverageReward(episodes) = 0;
    train.AverageSteps(episodes)  = 0;
    train.L(episodes) = 0;
    if episodes > (memory.capacity/parameter.maxit)
        parameter.sigma = parameter.sigma * parameter.noiseDecay;
        if parameter.sigma < 0.005
            parameter.sigma = 0.005;
        end
    end 
    for g = 1:parameter.maxit
        train.totalSteps = train.totalSteps + 1;
        if ~ishandle(h)
            h = figure(1);
            h.Position = hPosition;
            subplot(211)
            h1 = animatedline('Color','r','Marker','o');
            h2 = animatedline('Color','b','Marker','*');
            legend('EpisodeReward','AverageReward','Location','northwest');
            xlabel('Episodes'); ylabel('Reward');grid on;
            subplot(212);
            h3 = animatedline('Color','g','Marker','x');
            legend('Loss','Location','northwest');
            xlabel('Episodes'); ylabel('Loss');grid on;
        end
        if ishandle(h)
            hPosition = h.Position;
        end
        
        tempt = state0_o + action0_o;
        control.image.netKu = [train.gainKp train.gainKi train.gainKd].*nnKu.forward([state0_o(:,2:4) action0_o(:,2:4)]);
        train.noise = parameter.sigma.*randn(1,3);
        control.image.netKu = min(max(control.image.netKu + train.noise,[0 0 0]),[train.gainKp train.gainKi train.gainKd]);

        control.image.netKw = [train.gainKp train.gainKi train.gainKd].*nnKw.forward([state0_o(:,6:8) action0_o(:,6:8)]);
        train.noise = parameter.sigma.*randn(1,3);
        control.image.netKw = min(max(control.image.netKw + train.noise,[0 0 0]),[train.gainKp train.gainKi train.gainKd]);
        
        control.image.Kp = [control.image.netKu(1) 0;0 control.image.netKw(1)];
        control.image.Ki = [control.image.netKu(2) 0;0 control.image.netKw(2)];
        control.image.Kd = [control.image.netKu(3) 0;0 control.image.netKw(3)];
        
        [gimbal,control ] = sceneSteering( tempt,image,gimbal,control,environment);
        
        % STEP DYNAMICS FORWARD
        % simulatorDelay;
        [image,control,gimbal,train,state1_o,action1_o] = simulatorDelay(image,control,gimbal,train,environment,state0_o,action0_o);
        if norm([state1_o(2:3),state1_o(6:7)]) < train.done && episodes > (memory.capacity/parameter.maxit)
            terminal = 1;
        else
            terminal = 0;
        end
        memory.index = memory.index + 1;
        
        % Starage Data
        indexs = mod(memory.index,memory.capacity);
        if indexs == 0
            indexs = memory.capacity;
        end
        tempt1 = state1_o+action1_o;
        memory.data(:,indexs) = [state0_o,action0_o,[control.image.netKu,control.image.netKw],rewardFunc([tempt1(3),tempt1(7)],[tempt1(2),tempt1(6)]),state1_o,action1_o,terminal]';
        train.EpisodeReward(episodes) = train.EpisodeReward(episodes)+ rewardFunc([tempt1(3),tempt1(7)],[tempt1(2),tempt1(6)]); 
        state0_o = state1_o;
        action0_o = action1_o;
        % Train
        if memory.index > memory.capacity
            memory = batchSample (memory);
            
            train.O   = memory.train.data(1:16,:)';
            train.A   = memory.train.data(17:22,:)';
            train.R   = memory.train.data(23,:)';
            train.n_O = memory.train.data(24:39,:)';
            train.Term    = memory.train.data(end,:)';
            % Compute targets via Bellman equation with target network
            train.n_A  = [train.gainKp train.gainKi train.gainKd].*nnKut.forward([train.n_O(:,2:4) train.n_O(:,10:12)]);
            train.n_AI = [train.gainKp train.gainKi train.gainKd].*nnKwt.forward([train.n_O(:,6:8) train.n_O(:,14:16)]);
            train.n_Qt = forward(nnQt,[train.n_A train.n_AI train.n_O(:,2:4) train.n_O(:,6:8) train.n_O(:,10:12) train.n_O(:,14:16)]);
            train.T    = train.R + parameter.discount.* train.n_Qt.* ~train.Term;
            % Compute error, loss and gradients
            train.Q1 = forwardfull(nnQ,[train.A train.O(:,2:4) train.O(:,6:8) train.O(:,10:12) train.O(:,14:16)]);
            train.E = train.Q1-train.T;
            train.L(episodes) = train.L(episodes) + mean(train.E.^2) / memory.batch_size;
            train.dL = 2 * train.E / memory.batch_size;
            % Critic update (Q-networks)
            train.dW_q = nnQ.backward(train.dL);
            nnQ.update(optimQ.step(nnQ.W,train.dW_q));
            if mod(train.totalSteps,5)
                nnQt.update(parameter.tau * nnQ.W + (1 - parameter.tau) * nnQt.W);
            end
            % Actor update (policy networks)
            train.A_det  = [train.gainKp train.gainKi train.gainKd].*nnKu.forwardfull([train.O(:,2:4) train.O(:,10:12)]);
            train.AI_det = [train.gainKp train.gainKi train.gainKd].*nnKw.forwardfull([train.O(:,6:8) train.O(:,14:16)]); 
            train.Q_det  = forwardfull(nnQ,[train.A_det train.AI_det train.O(:,2:4) train.O(:,6:8) train.O(:,10:12) train.O(:,14:16)]);
            [~, train.dL_q] = backward(nnQ,ones(size(train.Q_det))); 
            train.dL_qq = train.dL_q(:,1:3);
            train.dW_p = - backward(nnKu,[train.gainKp train.gainKi train.gainKd].*train.dL_qq);
            nnKu.W = step(optimKu, nnKu.W, train.dW_p);
            if mod(train.totalSteps,5)
                nnKut.W = parameter.tau * nnKu.W + (1 - parameter.tau) * nnKut.W;
            end
            % Actor update (policy networks)
            train.dLI_q = train.dL_q(:,4:6);
            train.dWI_p = - backward(nnKw,[train.gainKp train.gainKi train.gainKd].*train.dLI_q);
            nnKw.W = step(optimKw, nnKw.W, train.dWI_p);
            if mod(train.totalSteps,5)
                nnKwt.W = parameter.tau * nnKw.W + (1 - parameter.tau) * nnKwt.W;
            end
        end
        % Check Done
        if terminal
            break;
        end
        
    end
    %% UPDATE PLOTS
    train.i = mod(episodes,5);
    if train.i == 0
        train.i = 5;
    end
    train.temptReward(train.i) = train.EpisodeReward(episodes);
    train.AverageReward(episodes) = sum(train.temptReward)/length(train.temptReward);
    train.temptSteps(train.i) = g;
    train.AverageSteps(episodes) = sum(train.temptSteps)/length(train.temptSteps);
    fprintf('***********************************\n');
    fprintf('Number of episodes = %d\n',episodes);
    fprintf('EpisodeReward = %6.1f\n',train.EpisodeReward(episodes));
    fprintf('AverageReward = %6.1f\n',train.AverageReward(episodes));
    fprintf('Loss          = %3.3f\n',train.L(episodes)/g);
    fprintf('EpisodeSteps  = %6.1f\n',g);
    fprintf('AverageSteps  = %6.1f\n',train.AverageSteps(episodes));
    fprintf('TotalSteps    = %6.1f\n',train.totalSteps);
    fprintf('Sigma         = %1.1f\n',parameter.sigma);
    if terminal
       fprintf('Done\n');
    end
    addpoints(h1,episodes,train.EpisodeReward(episodes));
    addpoints(h2,episodes,train.AverageReward(episodes));
    addpoints(h3,episodes,train.L(episodes)/g);
    drawnow;

    %% Stop Training
    if train.AverageReward(episodes) > train.stopReward && episodes > (memory.capacity/parameter.maxit) || episodes == parameter.maxEpi
        fprintf('Train Done\n');
        Test;
        break;
    end  
end