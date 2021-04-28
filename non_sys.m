function dx = non_sys(x,x_bar,k)

%    centralization and calculate control
   y = x(1:2);
   x_hat = y-x_bar;
   u = -k(1:2,1:2)*x_hat(1:2);
   
%    progress system
   theta1 = x(3);
   theta2 = x(4);
   L1 = 0.5;
   L2 = 0.5;

%    get the change of the system
   dx(1,1) = (-L1*sin(theta1)-L2*sin(theta1+theta2))*u(1) + (-L2*sin(theta1+theta2))*u(2);
   dx(2,1) = (L1*cos(theta1)+L2*cos(theta1+theta2))*u(1) + ( L2*cos(theta1+theta2))*u(2);
   dx(3,1) = u(1);
   dx(4,1) = u(2);
   
end