% Before running this script follow the instructions on http://petercorke.com/wordpress/toolboxes/robotics-toolbox
% Download RTB-10.3.1 mltbx format (23.2 MB) in MATLAB toolbox format (.mltbx)
% From within the MATLAB file browser double click on this file, it will install and configure the paths correctly
% Download RTB-10.3.1 zip format (22.5 MB) as a zip file (.zip)
% Unpack the archive which will create the directory (folder) rvctools, and within that the directories robot, simulink, and common.
% Adjust your MATLABPATH to include rvctools
% Execute the startup file rvctools/startup_rvc.m and this will place the correct directories in your MATLAB path.

% For IRB1600
alphas_d = [ -90, 0, -90, 90, -90, 0 ];
alphas_r = alphas_d * pi / 180;
ds = [ 10, 10, 10 ];
as = [ 10, 10 ];
thetas_d = [ 90, 0, 0, 0, 0, 45 ];
thetas_r = thetas_d * pi / 180;

% (a)
poseByUs = forwardKinematics(thetas_d, as, ds, alphas_d);
disp('Pose of end effector given by our function:');
display(poseByUs)

% (b)
% Defining robot in robot tool box
L(1) = Revolute('d', ds(1), 'a', as(1), 'alpha', alphas_r(1));
L(2) = Revolute('d',     0, 'a', as(2), 'alpha', alphas_r(2));
L(3) = Revolute('d',     0, 'a',     0, 'alpha', alphas_r(3));
L(4) = Revolute('d', ds(2), 'a',     0, 'alpha', alphas_r(4));
L(5) = Revolute('d',     0, 'a',     0, 'alpha', alphas_r(5));
L(6) = Revolute('d', ds(3), 'a',     0, 'alpha', alphas_r(6));
IRB1600 = SerialLink(L, 'name', 'IRB1600');

poseByFKINE = IRB1600.fkine(thetas_r);
disp('Pose of end effector given by fkine():');
display(poseByFKINE)

% (c)
disp('Original joint angles:');
display(thetas_d)

jointAngles_d = inverseKinematics(poseByUs, as, ds);
disp('Joint angles predicted by us:');
display(jointAngles_d)
IRB1600.fkine(jointAngles_d * pi / 180)

% (d)
disp('Joint angles given by ikine():');
jointAngles = IRB1600.ikine(poseByUs);
display(jointAngles * 180 / pi)
IRB1600.fkine(jointAngles)

jointAngles = IRB1600.ikine(poseByUs, [ pi / 2, 0, 0, 0, 0, pi / 4 ]);
display(jointAngles * 180 / pi)
