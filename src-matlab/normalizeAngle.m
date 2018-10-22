function normalizedAngle = normalizeAngle( angle )
% normalizes a given angle to a range between -pi ... pi
normalizedAngle = mod( angle + pi, 2*pi ) - pi;
