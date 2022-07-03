function simulation = without_control(t,y)
global a b c r v_bar;

simulation = [0
              y(1)
              ((r+1)*y(4)-(r+1)*v_bar*y(1))/a
              y(3)
              y(6)/a
              y(5)];