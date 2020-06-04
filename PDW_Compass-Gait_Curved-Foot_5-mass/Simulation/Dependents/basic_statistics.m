function [struct] = basic_statistics(struct, label, data)
%BASIC_STATISTICS Summary of this function goes here
%   Detailed explanation goes here

    % Basic Statistics
    struct.(label).mean   = mean(data);
    struct.(label).std    = std(data);
    struct.(label).median = median(data);
    struct.(label).min    = min(data);
    struct.(label).max    = max(data);
end

