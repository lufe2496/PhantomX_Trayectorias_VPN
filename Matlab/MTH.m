function  MTH = MTH(pos)
% Convertir Posicion y Orientación en x,y,z,r,p,y en MTH
% Esta función usa las funciones del toolbox de Peter Corke
          MTH = transl(pos(1:3))*rpy2tr(pos(4:6),'deg');          
          
end