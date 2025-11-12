function M = motorCommandConvert(m_command)
% NEW!!! To convert command data into combination of High eight and Low eight
M = zeros(3,2);

for i = 1:3
    if (m_command(i) < 0)
        disp('Warning, cannot increase curvature further! Need to decrease arc length!')
        m_command(i) = 0;
    end
   M(i,1) = bitshift(m_command(i),-8);
   M(i,2) = bitand(m_command(i),255);
end
end
