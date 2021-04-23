function T_out=T(eul) 

 T_out=[1 sin(eul(1))*tan(eul(2)) -sin(eul(2));...
     0 cos(eul(1)) -sin(eul(1));...
     0 sin(eul(1))/sin(eul(2)) cos(eul(1))/cos(eul(2))];

end