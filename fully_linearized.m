function simulation = fully_linearized(t,y)
global a b c r v_bar b_bar b1 b2 b3 b4 b5 b6;

simulation = [(r+1)*b_bar*b1*y(1)/a+(r+1)*(b_bar*b2+1)*y(2)/a+(r+1)*b_bar*b3*y(5)/a
              y(1)
              -(r+1)*b4*y(3)/a+(1-(r+1)*b5)*y(4)/a-(r+1)*b6*y(6)/a
              y(3)
              b1*y(1)+b2*y(2)+b3*y(5)
              b4*y(3)+b5*y(4)+b6*y(6)];