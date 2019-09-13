function [angle] =  normalize_angle_f(r,low)
% Compute the angle in the semi-closed interval [low, low + 2PI) that represents the same rotation as the angle <em>r</em>
% r   angle in radians
% low starting value of the normalization interval
% angle = equivalent angle in the interval [low, low + 2PI) computed as r - 2PI * FLOOR((r - low) / (2PI)) 

angle = r - 2*pi*floor((r - low)/(2*pi));