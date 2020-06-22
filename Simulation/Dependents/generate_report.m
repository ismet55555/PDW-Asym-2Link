function [true] = generate_report(p, results)
%GENERATE_REPORT Summary of this function goes here
%   Detailed explanation goes here

% Output to command window
if (results.sim.stopped == true)
   fprintf('\n\nWalker successfully completed simulation.\n') 
else
   % TODO: 
   %    Include number of steps of total steps, how it failed, in what phase.
   fprintf('Walker failed to complete simulation.\n') 
end
        

end

