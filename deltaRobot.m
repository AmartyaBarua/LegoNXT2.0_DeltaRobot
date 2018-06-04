pause on
COM_CloseNXT('all');
close all;
clear all;
clc;

h = COM_OpenNXT();      % handle for the controller
COM_SetDefaultNXT(h);   % set h to be the default
NXT_PlayTone(440,100)   % connection check (plays a tone if established)

% declaring motors and associated ports etc.
mA = NXTMotor('A');
mA.SpeedRegulation = true;
mA.ActionAtTachoLimit = 'HoldBrake';
mA.SmoothStart = true;

mB = NXTMotor('B');
mB.SpeedRegulation = true;
mB.ActionAtTachoLimit = 'HoldBrake';
mB.SmoothStart = true;

mC = NXTMotor('C');
mC.SpeedRegulation = true;
mC.ActionAtTachoLimit = 'HoldBrake';
mC.SmoothStart = true;

mA.Stop('off');   % stopping motor if engaged previously
mB.Stop('off');
mC.Stop('off');

power = 20;     % driving power (configureable)
reverse_power = -1 * power;

mA.ResetPosition();   % reset the tachometer
mB.ResetPosition();
mC.ResetPosition();

N1 = 12;    % number of teeth of the pinion
N2 = 36;    % number of teeth of gear
gear_ratio = -N2 / N1;

% Route
% start at the center
position1 = [0 0 -32];
ang11 = inverse(position1);     % calculate the input angles
ang21 = gear_ratio .* ang11;

% read degrees from the sensor
dataA = mA.ReadFromNXT();
posA = dataA.Position;
dataB = mB.ReadFromNXT();
posB = dataB.Position;
dataC = mC.ReadFromNXT();
posC = dataC.Position;

% error
a = round(ang21(1) + posA);
b = round(ang21(2) + posB);
c = round(ang21(3) + posC);
angles0 = [a b c];

% check which way is reverse
if angles0(1) > posA
  mA.Power = reverse_power;
else
  mA.Power = power;
end
mA.TachoLimit = abs(angles0(1));
if angles0(2) > posB
  mB.Power = reverse_power;
else
  mB.Power = power;
end
mB.TachoLimit = abs(angles0(2));
if angles0(3) > posC
  mC.Power = reverse_power;
else
  mC.Power = power;
end
mC.TachoLimit = abs(angles0(3));

% send the angles to the motors
mA.SendToNXT();
mB.SendToNXT();
mC.SendToNXT();
mA.WaitFor();     % wait until the current task is completed
mB.WaitFor();
mC.WaitFor();

% read degrees from the sensor
dataA = mA.ReadFromNXT();
posA = dataA.Position;
dataB = mB.ReadFromNXT();
posB = dataB.Position;
dataC = mC.ReadFromNXT();
posC = dataC.Position;

mA.Power = - mA.Power;
mB.Power = - mB.Power;
mC.Power = - mC.Power;
mA.TachoLimit = abs(posA);
mB.TachoLimit = abs(posB);
mC.TachoLimit = abs(posC);
pause     % wait for user input
mA.Stop('off');
mB.Stop('off');
mC.Stop('off');
mA.SendToNXT();
mB.SendToNXT();
mC.SendToNXT();
mA.WaitFor();
mB.WaitFor();
mC.WaitFor();
mA.Stop('off');
mB.Stop('off');
mC.Stop('off');

% Destination (can be changed)
X = 0;
Y = -7;
Z = -22;
position2 = [X Y Z];
ang12 = inverse(position2);
ang22 = gear_ratio .* ang12;

% read degrees from the sensor
dataA1 = mA.ReadFromNXT();
posA1 = dataA1.Position;
dataB1 = mB.ReadFromNXT();
posB1 = dataB1.Position;
dataC1 = mC.ReadFromNXT();
posC1 = dataC1.Position;

% error
a1 = round(ang22(1) + posA1);
b1 = round(ang22(2) + posB1);
c1 = round(ang22(3) + posC1);
angles1 = [a1 b1 c1];

% check which way is reverse
if angles1(1) > posA1
  mA.Power = reverse_power;
else
  mA.Power = power;
end
mA.TachoLimit = abs(angles1(1));
if angles1(2) > posB1
  mB.Power = reverse_power;
else
  mB.Power = power;
end
mB.TachoLimit = abs(angles1(2));
if angles1(3) > posC1
  mC.Power = reverse_power;
else
  mC.Power = power;
end
mC.TachoLimit = abs(angles1(3));

% send angles to the motors
mA.Stop('off');
mB.Stop('off');
mC.Stop('off');
mA.SendToNXT();
mB.SendToNXT();
mC.SendToNXT();
mA.WaitFor();
mB.WaitFor();
mC.WaitFor();

% measure the angles
dataA2 = mA.ReadFromNXT();
posA2 = dataA2.Position;
dataB2 = mB.ReadFromNXT();
posB2 = dataB2.Position;
dataC2 = mC.ReadFromNXT();
posC2 = dataC2.Position;

% return to original position
mA.Power = - mA.Power;
mB.Power = - mB.Power;
mC.Power = - mC.Power;
mA.TachoLimit = abs(posA2);
mB.TachoLimit = abs(posB2);
mC.TachoLimit = abs(posC2);
pause
mA.Stop('off');
mB.Stop('off');
mC.Stop('off');
mA.SendToNXT();
mB.SendToNXT();
mC.SendToNXT();
mA.WaitFor();
mB.WaitFor();
mC.WaitFor();
mA.Stop('off');
mB.Stop('off');
mC.Stop('off');
