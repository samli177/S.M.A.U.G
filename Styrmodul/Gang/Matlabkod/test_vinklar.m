function [x,y,z] = test_vinklar( x,y,z )

coxa = 56;
femur = 66;
tibia = 131;




    gamma = atan(x/y);
    d = sqrt(z^2+(x-coxa*sin(gamma))^2+(y-coxa*cos(gamma))^2);
    beta = pi - acos((femur^2+tibia^2-d^2)/(2*femur*tibia));
    alpha = acos((femur^2-tibia^2+d^2)/(2*femur*d))-asin(z/d);
    
x=sin(gamma)*(coxa+femur*cos(alpha)+tibia*cos(-beta+alpha));
y=cos(gamma)*(coxa+femur*cos(alpha)+tibia*cos(-beta+alpha));
z=-femur*sin(alpha)+tibia*sin(beta-alpha);


end

