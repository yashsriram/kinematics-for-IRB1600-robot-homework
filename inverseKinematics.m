function [ jointAngles ] = inverseKinematics( Transformation_given, as, ds )

x = Transformation_given(1,4);
y = Transformation_given(2,4);
z = Transformation_given(3,4);
R= Transformation_given(1:3,1:3);

xc = x - ds(3)*R(1,3);
yc = y - ds(3)*R(2,3);
zc = z - ds(3)*R(3,3);
r = sqrt(xc^2 + yc^2);

%Inverse kinematic equations for position
theta1 = atan2(yc,xc);

r = (xc^2 + yc^2)^(1/2);
D = ((r - as(1))^2 + (zc - ds(1))^2 - as(2)^2 - ds(2)^2)/(2*as(2)*ds(2));

theta3_dash = atan2((-(1 - D^2)^(1/2)), D);
theta3 = abs(theta3_dash) - (90*pi/180);

theta2_dash = (atan2( zc-ds(1), r-as(1)) - atan2(ds(2)*sind(theta3_dash*180/pi),as(2)+ds(2)*cosd(theta3_dash*180/pi))) ;
theta2 = (90*pi/180) - theta2_dash;

theta5 = atan2(sqrt(1-(sin(theta1)*R(1,3)- cos(theta1)*R(2,3))^2),sin(theta1)*R(1,3) - cos(theta1)*R(2,3)) -pi;
theta4 = atan2(-cos(theta1)*sin(theta2+theta3)*R(1,3)-sin(theta1)*sin(theta2+theta3)*R(2,3)+cos(theta2+theta3)*R(3,3),cos(theta1)*cos(theta2+theta3)*R(1,3)+sin(theta1)*cos(theta2+theta3)*R(2,3)+sin(theta2+theta3)*R(3,3));
theta6= atan2(sin(theta1)*R(1,2)-cos(theta1)*R(2,2),-sin(theta1)*R(1,1)+cos(theta1)*R(2,1));

%Converting angles to degrees, because angles passed as arguments to the
%function forwardKinematics() have to be in degrees.
theta1 = theta1 * 180 / pi;
theta2 = theta2 * 180 / pi;
theta3 = theta3 * 180 / pi;
theta4 = theta4 * 180 / pi;
theta5 = theta5 * 180 / pi;
theta6 = theta6 * 180 / pi;

jointAngles = [theta1, theta2, theta3, theta4, theta5, theta6];

end