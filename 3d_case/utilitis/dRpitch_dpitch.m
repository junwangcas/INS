function [dRpitch_dpitch] = dRpitch_dpitch(pitch)
dRpitch_dpitch = [-sin(pitch),0,cos(pitch);
           0, 0, 0;
          -cos(pitch),0,sin(pitch)];
end

