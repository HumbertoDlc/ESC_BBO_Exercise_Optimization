% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Generate animation of 2 DOF linkage tracing ellipse

function animate(q1,q2,t,decFctr,xShape,yShape,l1,l2,ox,oy,animHdl,animColor)

% reduce inputs
t = t(1:decFctr:end);
q1 = q1(1:decFctr:end)';
q2 = q2(1:decFctr:end)';

% compute joint locations
xvals = [ox*ones(length(q1),1) ox+l1*cos(q1) ox+l1*cos(q1)+l2*cos(q1+q2)];
yvals = [oy*ones(length(q1),1) oy+l1*sin(q1) oy+l1*sin(q1)+l2*sin(q1+q2)];

% plot each frame
figure(animHdl)
plot(xShape,yShape,animColor)
hold on
for idx = 1:length(t)
    h = plot(xvals(idx,:),yvals(idx,:),['-o',animColor]);
    axis equal
    title(['t = ' num2str(t(idx))])
    pause(0.1)
    if idx < length(q1)
        delete(h)
    end
end

end