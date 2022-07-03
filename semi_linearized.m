function simulation = semi_linearized(t,y)
global a b c r v_bar b_bar b1 b2 b3 b4 b5 b6;

M = [c 0 b*b1 0 0 0 0 0
     0 1 0 0 0 0 0 0
     0 0 y(4)*b*b1 0 a 0 0 0
     0 0 0 1 0 0 0 0
     0 0 a-y(6)*b*b1 0 0 0 0 0
     0 0 0 0 0 1 0 0
     0 0 0 0 0 0 1 0
     0 0 0 0 0 0 0 1];

N = [-b*(b1*b3+b2)*y(3)-b*b2*b3*y(4)-b*(b3^2)*y(7)+(r+1)*y(4)*(b4*y(5)+b5*y(6)+b6*y(8))
     y(1)
     -b*b1*y(3)^2-b*((b1*b3+2*b2)*y(4)+b3*y(7))*y(3)-(r+1)*b4*y(5)-b*b2*b3*y(4)^2-b*b3*b3*y(4)*y(7)-(b5*r+b5-1)*y(6)-b6*(r+1)*y(8)
     y(3)
     -(-b*b1*y(3)+b4*(r+1)*y(2)-b*(b3*y(7)+b2*y(4)))*y(5)-(r+1)*(v_bar+y(8))*y(1)+b*(b1*b3+b2)*y(6)*y(3)-(b5*(r+1)*y(2)-b*b3*(b3*y(7)+b2*y(4)))*y(6)-(r+1)*(y(2)*y(8)*b6-y(4))
     y(5)
     b1*y(3)+b2*y(4)+b3*y(7)
     b4*y(5)+b5*y(6)+b6*y(8)];

simulation = M\N;