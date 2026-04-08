clear all; close all; clc;
global t_plot count
ini;
main_part;
while t_plot(count-1)<max(t_plot);
    main_part;
end
Ploting;%Вывод графиков

function ini
global ts g b d l m x y z dx dy dz ix iy iz
global phi thet psi dphi dthet dpsi t_plot count
%%%%%%%%%%Параметры%%%%%%%%%%
time=100;%Время симуляции
ts=0.01;%Шаг симуляции
t_plot=[0:ts:time];%Ось времени
g=9.8;%Ускорение свободного падения
b=0.2;%Коэф тяги
d=1;%Коэф лобового сопротивления
l=0.15;%Длина от центра до ротора
m=4;%Масса дрона

%%%%%%%%%%Изначальное состояние%%%%%%%%%%
%Координаты
x=0; y=0; z=0;
%Линейная скорость
dx=0; dy=0; dz=0;
%Угол
phi=0;%Крен
thet=0;%Тангаж
psi=0;%Рысканье
%Угловая скорость
dphi=0; dthet=0; dpsi=0;
%Инерция
ix=0.007; iy=0.005; iz=0.001;

%%%%%%%%%%Желаемое состояние%%%%%%%%%%
global finx finy finz finphi finthet finpsi
%Координаты
finx=0;
finy=0;
finz=5;
%Угол
finphi=0;%Крен
finthet=0;%Тангаж
finpsi=0;%Рысканье

global n1 n2 n3 n4 Omeg
%Количество оборотов роторов в минуту
n1=120;
n2=120;
n3=120;
n4=120;

count=1;%Счётчик цикла
end

function Angle_rotor %Расчёт угловых скоростей роторов
  global n1 n2 n3 n4 Omeg
  omeg1=2*pi*n1/60;
  omeg2=2*pi*n2/60;
  omeg3=2*pi*n3/60;
  omeg4=2*pi*n4/60;
  Omeg=[omeg1; omeg2; omeg3; omeg4];
end

function main_part %Преобразования
  global  ts g b d l m x y z dx dy dz phi thet psi dphi dthet dpsi 
  global ix iy iz count Omeg
  global x_plot y_plot z_plot phi_plot thet_plot psi_plot
  global n1 n2 n3 n4 n1_plot n2_plot n3_plot n4_plot
  global dx_plot dy_plot dz_plot dphi_plot dthet_plot dpsi_plot
  Stabil;
  Angle_rotor;
  %Суммарная тяга роторов и крутящие моменты
  ft=b*(Omeg(1)^2 + Omeg(2)^2 + Omeg(3)^2 + Omeg(4)^2);
  tx=b*l*(Omeg(3)^2 - Omeg(1)^2);
  ty=b*l*(Omeg(4)^2 - Omeg(2)^2);
  tz=d*(Omeg(2)^2 + Omeg(4)^2 - Omeg(1)^2 - Omeg(3)^2);
  %Линейное ускоение
  ddx = (ft/m)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(thet));
  ddy = (ft/m)*(cos(phi)*sin(psi)*sin(thet) - cos(psi)*sin(phi));
  ddz = -g + ((ft/m)*(cos(phi)*cos(thet)));
  %Угловое ускорение
  ddphi = (((iy-iz)/ix) * dthet*dpsi) + (tx/ix);%Крен
  ddthet = (((iz-ix)/iy) * dphi*dpsi) + (ty/iy);%Тангаж
  ddpsi = (((ix-iy)/iz) * dphi*dthet) + (tz/iz);%Рысканье
  
  %Обновлённые линейные скорости
  dx = (ddx*ts) + dx;
  dy = (ddy*ts) + dy;
  dz = (ddz*ts) + dz;
  %Обновлённые координаты
  x = (dx*ts) + x;
  y = (dy*ts) + y;
  z = (dz*ts) + z;
  %Обновлённые угловые скорости
  dphi = (ddphi*ts) + dphi;
  dthet =(ddthet*ts) + dthet;
  dpsi = (ddpsi*ts) + dpsi;
  %Обновлённые углы
  phi = (dphi*ts) + phi;
  thet = (dthet*ts) + thet;
  psi = (dpsi*ts) + psi;

  %Запись данных для графиков
  x_plot(count)=x;
  y_plot(count)=y;
  z_plot(count)=z;
  phi_plot(count)=phi;
  thet_plot(count)=thet;
  psi_plot(count)=psi;

  %Количество оборотов в минуту в момент времени
  n1_plot(count)=n1;
  n2_plot(count)=n2;
  n3_plot(count)=n3;
  n4_plot(count)=n4;

  %Линейная и угловая скорости
  dx_plot(count)=dx;
  dy_plot(count)=dy;
  dz_plot(count)=dz;
  dphi_plot(count)=dphi;
  dthet_plot(count)=dthet;
  dpsi_plot(count)=dpsi;

  count=count+1;
end

function Ploting
%Графики положения
global x_plot y_plot z_plot phi_plot thet_plot psi_plot t_plot
subplot(2,3,1);
plot(t_plot,x_plot);
title('Положение по Х');
xlabel('Время');
ylabel('Дистанция, м');
hold on; grid on;

subplot(2,3,2);
plot(t_plot,y_plot);
title('Положение по У');
xlabel('Время');
ylabel('Дистанция, м');
hold on; grid on;

subplot(2,3,3);
plot(t_plot,z_plot);
title('Положение по Z');
xlabel('Время');
ylabel('Дистанция, м');
hold on; grid on;

subplot(2,3,4);
plot(t_plot,phi_plot);
title('Крен');
xlabel('Время');
ylabel('Угол, рад');
hold on; grid on;

subplot(2,3,5);
plot(t_plot,thet_plot);
title('Тангаж');
xlabel('Время');
ylabel('Угол, рад');
hold on; grid on;

subplot(2,3,6);
plot(t_plot,psi_plot);
title('Рысканье');
xlabel('Время');
ylabel('Угол, рад');
hold on; grid on;

%Графики количества оборотов двигателей
global n1_plot n2_plot n3_plot n4_plot
figure;
subplot(2,2,1);
plot(t_plot,n1_plot);
title('Двигатель 1')
hold on; grid on;

subplot(2,2,2);
plot(t_plot,n2_plot);
title('Двигатель 2')
hold on; grid on;

subplot(2,2,3);
plot(t_plot,n3_plot);
title('Двигатель 3')
hold on; grid on;

subplot(2,2,4);
plot(t_plot,n4_plot);
title('Двигатель 4')
hold on; grid on;

%Графики линейной и угловой скоростей
global dx_plot dy_plot dz_plot dphi_plot dthet_plot dpsi_plot

figure;
subplot(2,3,1);
plot(t_plot,dx_plot);
title('Скорость по Х');
xlabel('Время');
ylabel('Скорость, м/с');
hold on; grid on;

subplot(2,3,2);
plot(t_plot,dy_plot);
title('Скорость по Y');
xlabel('Время');
ylabel('Скорость, м/с');
hold on; grid on;

subplot(2,3,3);
plot(t_plot,dz_plot);
title('Скорость по Z');
xlabel('Время');
ylabel('Скорость, м/с');
hold on; grid on;

subplot(2,3,4);
plot(t_plot,dphi_plot);
title('Скорость крена');
xlabel('Время');
ylabel('Угловая скорость, рад/c');
hold on; grid on;

subplot(2,3,5);
plot(t_plot,dthet_plot);
title('Скорость тангажа');
xlabel('Время');
ylabel('Угловая скорость, рад/c');
hold on; grid on;

subplot(2,3,6);
plot(t_plot,dpsi_plot);
title('Скорость рысканья');
xlabel('Время');
ylabel('Угловая скорость, рад/c');
hold on; grid on;
end

function Stabil %Стабилизация
global x y z phi thet psi dx dy dz dphi dthet dpsi n1 n2 n3 n4 finx finy finz finphi finthet finpsi
% if z>finz
%     n1=n1-20;
%     n2=n2-20;
%     n3=n3-20;
%     n4=n4-20;
% else
%     n1=n1+20;
%     n2=n2+20;
%     n3=n3+20;
%     n4=n4+20;
% end

end