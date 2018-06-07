fuction [THETAS] = inverseKinematics(position)

X = position(1);
Y = position(2);
Z = position(3);

% Geometry of the delta robot
E = 22.5;    % length of the side of the end effector (change this if needed)
F = 45;      % length of the side of the base frame (change this if needed)
RE = 29.5;   % length of the shin (change this if needed)
RF = 7.5;    % lenght of the thigh (change this if needed)
R3 = sqrt(3);
E2 = E/2;
F2 = F/2;
E4 = E/4;
F4 = F/4;

% Plucker Coordinate
C1X = X;
C1Y = Y - (E2/R3)
C1Z = Z;
C2X = X + E4;
C2Y = Y + (E4/R3);
C2Z = Z;
RF2 = RF^2;
C3X = X - E4;
C3Y = C2Y;
C3Z = Z;
D1Y = -F2/R3;
D2X = F4;
D2Y = F4/R3;
D3X = -F4;
D3Y = D2Y;
EF = (RE^2) - RF2;
X1 = C1X;
Y1 = C1Y - D1Y;
Z1 = C1Z;
W1 = (EF - (C1X^2) + (D1Y^2) - (C1Y^2) - (C1Z^2) - (C1Z^2)) / 2;
X2 = C2X - D2X;
Y2 = C2Y - D2Y;
Z2 = C2Z;
W2 = (EF + (D2X^2) - (C2X^2) + (D2Y^2) - (C2Y^2) - (C2Z^2)) / 2;
X3 = C3X - D3X;
Y3 = C3Y - D3Y;
Z3 = C3Z;
W3 = (EF + (D3X^2) - (C3X^2) + (D3Y^2) - (C3Y^2) - (C3Z^2)) / 2;
P02 = Z1;
P03 = -Y1;
P23 = W1;
Q01 = -R3 * Z2;
Q02 = -Z2;
Q03 = Y2 + (R3 * X2);
Q23 = -W2;
Q31 = R3 * W2;
R01 = R3 * Z3;
R02 = -Z3;
R03 = Y3 - (R3 * X3);
R23 = - W3;
R31 = -R3 * W3;

RD = 180 / pi;

% Find Theta 1;
T1 = P02 / P03;
U1 = P23 / P03;
A1 = T1^2+1;
B1 = T1 * (D1Y - U1);
C1 = U1 * (2*D1Y -U1) - (D1Y^2) + RF2;
D1 = B1^2 + (A1 * C1);
if D1 < 0
  disp('D1 is smaller than 0')
else
  D11 = sqrt(D1);
  V1 = (B1 - D11) / A1;
  S1 = V1 / RF;
  THETA01 = RD * atan(S1 / sqrt(1 - s1^2));
end

% Find Theta 2
T2 = Q02 / Q03;
U2 = Q01 / Q03;
V2 = Q31 / Q03;
W2 = Q23 / Q03;
A2 = T2^2 + U2^2 + 1;
B2 = U2 * (D2X + V2) + T2 * (D2Y - W2);
C2 = RF2 - D2X * (D2X + 2 * V2) - D2Y * (D2Y - 2 * W2) - (V2^2) - (W2^2);
D2 = B2^2 + A2 * C2;
if D2 < 0
  disp('D2 is smaller than 0')
else
  D22 = sqrt(D2);
  V22 = (B2 - D22) / A2;
  S2 = V22 / RF;
  THETA02 = RD * atan(S2 / sqrt(1-S2^2));
end

% Find Theta 3
T3 = R02 / R03;
U3 = R01 / R03;
V3 = R31 / R03;
W3 = R23 / R03;
A3 = T3^2 + U3^2 + 1;
B3 = U3 * (D3X + V3) + T3 * (D3Y - W3);
C3 = RF2 - D3X * (D3X + 2 * V3) - D3Y * (D3Y - 2 * W3) - V3^2 - W3^2;
D3 = B3^2 + (A3 *C3);
if D3 < 0
  disp('D3 is smaller than 0')
else
  D33 = sqrt(D3);
  V33 = (B3 - D33) / A3;
  S3 = V33 / RF;
  THETA03 = RD * atan(S3 / sqrt(1 - S3^2));
end

THETAS = [THETA01 THETA02 THETA03];
