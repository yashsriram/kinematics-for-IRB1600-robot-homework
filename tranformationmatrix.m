function [ transformation_matrix ] = tranformationmatrix(a, alpha, th, d)
% The angles should be input in "degrees"
% a     = link length
% aplha = link twist
% th    = link angle
% d     = link offset

transformation_matrix = [cosd(th), -sind(th)*cosd(alpha),  sind(th)*sind(alpha), a*cosd(th);
                         sind(th),  cosd(th)*cosd(alpha), -cosd(th)*sind(alpha), a*sind(th);
                              0  ,        sind(alpha)   ,      cosd(alpha)     ,      d    ;
                              0  ,           0          ,           0          ,       1   ];

end
