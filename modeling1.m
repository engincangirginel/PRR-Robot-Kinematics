%---------------------------------------------------------
% PRR Robot Kinematic Analysis
% Student Name: [Engin Can GİRGİNEL]
% Description: This script performs forward and inverse kinematics using
% MATLAB R2025b Robotics System Toolbox.
%---------------------------------------------------------
clear; clc; close all;
... (Kodun Geri Kalanı) ...
% --- TEMİZLİK ---
clear; clc; close all;

% --- 1. ROBOT MODELİNİ OLUŞTURMA (R2025b Native - rigidBodyTree) ---
robot = rigidBodyTree('DataFormat', 'row');

% --- Gövde 1: Prizmatik Eklem (d1) ---
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1', 'prismatic'); 
% DH: a=0, alpha=pi/2, d=değişken, theta=0
setFixedTransform(jnt1, [0, pi/2, 0, 0], 'dh');
body1.Joint = jnt1;
addBody(robot, body1, 'base');

% --- Gövde 2: Döner Eklem (theta2) ---
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2', 'revolute');
% DH: a=0, alpha=pi/2, d=0, theta=değişken
setFixedTransform(jnt2, [0, pi/2, 0, 0], 'dh');
body2.Joint = jnt2;
addBody(robot, body2, 'body1');

% --- Gövde 3: Döner Eklem (theta3) ---
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3', 'revolute');
% DH: a=20, alpha=0, d=0, theta=değişken (L3=20cm)
setFixedTransform(jnt3, [20, 0, 0, 0], 'dh');
body3.Joint = jnt3;
addBody(robot, body3, 'body2');

% --- Uç İşlevci (End Effector) ---
ee = rigidBody('end_effector');
setFixedTransform(ee.Joint, eye(4));
addBody(robot, ee, 'body3');

disp('Robot Modeli Başarıyla Oluşturuldu.');

% --- 2. İLERİ KİNEMATİK VE FIGURE 1 (Forward Kinematics) ---
% Sorudaki değerler: d1=15, theta2=30, theta3=60
d1_val = 15;
th2_val = 30 * pi/180;
th3_val = 60 * pi/180;

% Konfigürasyon vektörü
config_fk = [d1_val, th2_val, th3_val];

% Uç nokta matrisini hesapla
tform = getTransform(robot, config_fk, 'end_effector');

disp(' ');
disp('------------------------------------------------');
disp('a) İLERİ KİNEMATİK SONUCU (Figure 1):');
pos = tform(1:3, 4);
disp(['Uç Nokta Konumu (X, Y, Z): ', num2str(pos')]);
disp('------------------------------------------------');

% --- FIGURE 1 ÇİZİMİ ---
figure(1);
show(robot, config_fk);
title('Figure 1: İleri Kinematik (d1=15, \theta2=30, \theta3=60)');
axis([-30 30 -30 30 0 40]); % Eksen sınırları
grid on;
view(135, 30); % Daha iyi bir bakış açısı

% --- 3. TERS KİNEMATİK VE FIGURE 2 (Inverse Kinematics) ---
% Hedef Matris (Ttarget)
T_target = [
    0.3536 -0.6124  0.7071  10;
    0.3536 -0.6124 -0.7071  8;
    0.8660  0.5     0       12;
    0       0       0       1
];

% Ters kinematik çözücü
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.1 0.1 0.1 1 1 1]; % Pozisyona (son 3) öncelik ver
initial_guess = [0, 0, 0];

% Çözüm
[configSol, solInfo] = ik('end_effector', T_target, weights, initial_guess);

disp(' ');
disp('b) TERS KİNEMATİK SONUCU (Figure 2):');
disp(['d1 (cm):       ', num2str(configSol(1))]);
disp(['theta2 (derece): ', num2str(configSol(2) * 180/pi)]);
disp(['theta3 (derece): ', num2str(configSol(3) * 180/pi)]);

% --- FIGURE 2 ÇİZİMİ ---
figure(2);
show(robot, configSol);
title('Figure 2: Ters Kinematik Çözümü ve Hedef');
hold on;

% Hedef noktayı çiz (Hata düzeltilmiş hali)
rot_matrix = T_target(1:3, 1:3);
quat = rotm2quat(rot_matrix); % Quaternion dönüşümü
translation = T_target(1:3, 4)';

plotTransforms(translation, quat, 'FrameSize', 5); % Hedef koordinat sistemi
grid on;
view(135, 30);