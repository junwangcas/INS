function [dRroll_droll] = dRroll_droll(roll)
dRroll_droll = [0,0,0;
          0,-sin(roll),-cos(roll);
          0,cos(roll),-sin(roll)];
end

