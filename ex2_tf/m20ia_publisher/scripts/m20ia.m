q = zeros(6,1);

dh_params = [0.52, q(1), 0.15, pi/2;
             0, q(2)+pi/2, 0.79, 0;
             0, q(3), 0.15, pi/2;
             0.86, q(4), 0, pi/2;
             0, q(5), 0, -pi/2;
             0.1, q(6), 0, 0];

T01 = trvec2tform([0, 0, dh_params(1,1)])*trvec2tform([dh_params(1,3), 0, 0])*rotm2tform(rotz(rad2deg(dh_params(1,2))))*rotm2tform(rotx(rad2deg(dh_params(1,4))));
T12 = trvec2tform([0, 0, dh_params(2,1)])*trvec2tform([dh_params(2,3), 0, 0])*rotm2tform(rotz(rad2deg(dh_params(2,2))))*rotm2tform(rotx(rad2deg(dh_params(2,4))));
T23 = trvec2tform([0, 0, dh_params(3,1)])*trvec2tform([dh_params(3,3), 0, 0])*rotm2tform(rotz(rad2deg(dh_params(3,2))))*rotm2tform(rotx(rad2deg(dh_params(3,4))));
T34 = trvec2tform([0, 0, dh_params(4,1)])*trvec2tform([dh_params(4,3), 0, 0])*rotm2tform(rotz(rad2deg(dh_params(4,2))))*rotm2tform(rotx(rad2deg(dh_params(4,4))));
T45 = trvec2tform([0, 0, dh_params(5,1)])*trvec2tform([dh_params(5,3), 0, 0])*rotm2tform(rotz(rad2deg(dh_params(5,2))))*rotm2tform(rotx(rad2deg(dh_params(5,4))));
T56 = trvec2tform([0, 0, dh_params(6,1)])*trvec2tform([dh_params(6,3), 0, 0])*rotm2tform(rotz(rad2deg(dh_params(6,2))))*rotm2tform(rotx(rad2deg(dh_params(6,4))));

T46=T45*T56;
T36=T34*T46;
T26=T23*T36;
T16=T12*T26;
T06=T01*T16;
 
t06 = tform2trvec(T06);
R06 = tform2rotm(T06);
RPY06 = rotm2eul(R06); %Z-Y-X
aa06 = rotm2axang(R06);