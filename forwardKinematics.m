function [ pose ] = forwardKinematics( thetas_d, as, ds, alphas_d )
% DH-Parameters
% a     = link length
% th    = link angle
%   i       Link twist(alpha) Link Length (a) Link Offset (d)  Joint Angle (th)
% =========================================================================
%   1        -90              a1             d1                 theta1*     
%   2         0               a1              0               theta2 - 90*  
%   3        -90               0              0                 theta3*     
%   4         90               0             d4                 theta4*     
%   5        -90               0              0                 theta5*     
%   6         0                0             d5                 theta6*     

A_0_1 = tranformationmatrix(as(1),  alphas_d(1),      thetas_d(1),  ds(1)           );  
A_1_2 = tranformationmatrix(as(2),  alphas_d(2),      thetas_d(2),  0               );
A_2_3 = tranformationmatrix(    0,  alphas_d(3),      thetas_d(3),  0               );
A_3_4 = tranformationmatrix(    0,  alphas_d(4),      thetas_d(4),  ds(2)           ); 
A_4_5 = tranformationmatrix(    0,  alphas_d(5),      thetas_d(5),  0               );
A_5_6 = tranformationmatrix(    0,  alphas_d(6),      thetas_d(6),  ds(3)           );

pose = A_0_1 * A_1_2 * A_2_3 * A_3_4 * A_4_5 * A_5_6;

end