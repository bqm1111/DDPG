if episodes>0
    % Pendulum state:
    set(f,'XData',[0 -sin(z1(1))]);
    set(f,'YData',[0 cos(z1(1))]);
    [~,snewIdx] = min(sum((states - repmat(z1,[size(states,1),1])).^2,2));
    % Green tracer point:
    [newy,newx] = ind2sub([length(x2),length(x1)],snewIdx); % Find the 2d index of the 1d state index we found above
    set(pathmap,'XData',newx);
    set(pathmap,'YData',newy);
    % The heat map of best Q values
    V = max(Q,[],2); % Best estimated value for all actions at each state.
    fullV = reshape(V,[length(x2),length(x1)]); % Make into 2D for plotting instead of a vector.
    set(map,'CData',fullV);
    if transpMap
        set(map,'AlphaData',fullV~=Vorig); % Some spots have not changed from original. If not, leave them transparent.
    end
    drawnow;
    % Take a video frame if turned on.
end