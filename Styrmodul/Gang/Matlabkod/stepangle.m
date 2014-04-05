coxa = 56;
femur = 66;
tibia = 131;

y=200;
z=90;
x=-75:1:75;

for i=1:length(x)
    gamma(i) = atan(x(i)/y);
    d = sqrt(z^2+(x(i)-coxa*sin(gamma(i)))^2+(y-coxa*cos(gamma(i)))^2);
    beta(i) = pi - acos((femur^2+tibia^2-d^2)/(2*femur*tibia));
    alpha(i) = acos((femur^2-tibia^2+d^2)/(2*femur*d))-asin(z/d);
    
    
end

subplot(3,1,1)
plot(x,gamma)
title('gamma')
subplot(3,1,2)
plot(x,beta)
title('beta')
subplot(3,1,3)
plot(x,alpha)
title('alpha')