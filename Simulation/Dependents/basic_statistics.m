function [struct] = basic_statistics(struct, label, data)
% BASIC_STATISTICS: This is a utility script that computes basic statistic
%                   for the entered struct field

global log
log.debug(sprintf('Data: "%s": Computing basic statistics ... ', label))

% Basic Statistics
struct.(label).data   = data;
struct.(label).mean   = mean(data);
struct.(label).std    = std(data);
struct.(label).median = median(data);
struct.(label).min    = min(data);
struct.(label).max    = max(data);

end

