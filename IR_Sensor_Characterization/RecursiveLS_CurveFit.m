function [ xhat,yLS,RMS ] = RecursiveLS_CurveFit( t,y,N )
% define measurement covariance matrix R
R = 1;

% define initial guess at constants [x1:xN]
xhat = zeros(N+1,length(y));
if(N==1)
    x0 = [y(1); (y(end)-y(1))/(t(end)-t(1))];
else
    x0 = [y(1); ones(N,1)];
end

% define P0
P = (1e6)*eye(N+1);     % P describes initial confidence in your estimate
                        % No knowledge of x      -> P = (inf)*I
                        % Perfect knowledge of x -> P = 0

% solve recursive linear least-squares estimation
for(i=1:length(y))
    for(j=1:(N+1))
        if(j==1) H(1,j) = 1;
        else H(1,j) = (i-1)^(j-1);
        end
    end
    
    K = P*H'*inv(H*P*H' + R);   % Gain matrix
%   K = (H*P*H' + R)\(P*H');    % didn't work
    if(i==1) xhat(:,i) = x0 + K*(y(i)-H*x0);
    else xhat(:,i) = xhat(:,i-1) + K*(y(i)-H*xhat(:,i-1));
    end
    
    P = (eye(N+1) - K*H)*P*(eye(N+1) - K*H)' + K*R*K';
end

% define curve Nth order curve fit
yLS = zeros(1,length(y)+1);
for(i=1:(N+1))
    yLS = yLS + xhat(i,length(y))*t.^(i-1);
end

% determine Root Mean Square (RMS) Error
RMS = rms(y-yLS(1:length(y)));

end

