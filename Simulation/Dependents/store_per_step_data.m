function [struct] = store_per_step_data(struct, label, data, time, index)
% STORE_PER_STEP_DATA:   This script
%                           - Stores raw data
%                           - Computes interpolated vaues of raw data
%                           - Computes basic statistic of interpolated values
    

% Time vector
struct.(label).time{index} = time;

% Raw data vector
struct.(label).data{index} = data;

% Interpolate Data Linearly 
time_interp = linspace(0, time(end), 500);
data_interp = interp1(time, data, time_interp, 'linear', 'extrap');

struct.(label).time_interp         = time_interp;
struct.(label).data_interp{index}  = data_interp;

% Basic Statistics per step
struct.(label).mean(index)   = mean(data_interp);
struct.(label).std(index)    = std(data_interp);
struct.(label).median(index) = median(data_interp);
struct.(label).min(index)    = min(data_interp);
struct.(label).max(index)    = max(data_interp);

end
