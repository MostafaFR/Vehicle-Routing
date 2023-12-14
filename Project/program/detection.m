function [r_global, detect] = detection(sgi, sgI, yp, Yp, X)
    % Calculating the global residual

    yu = yp + sgi;
    Yu = Yp + sgI;

    Xkk = inv(Yu) * yu;
    Xkp = X;
    r_global = (Xkk-Xkp)' * Yu * (Xkk-Xkp);

    % Defining the detection threshold
    threshold = chi2inv(0.9, 3); % 90% confidence interval for a Chi-squared distribution with 3 degrees of freedom

    % Checking if the global residual exceeds the threshold
    if r_global > threshold
        detect = true; % Anomaly detected
    else
        detect = false; % No anomaly detected
    end
end