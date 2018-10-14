function normalizedAngle = normalizeAngle( angle )
% normalizes an given angle to a range between -pi ... pi
normalizedAngle = mod( angle + pi, 2*pi ) - pi;
