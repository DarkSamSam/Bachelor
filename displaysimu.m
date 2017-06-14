function displaysimu(t,y,u,filename)   %plots the simulation in graphs
%INPUT:
%   t: vector of sampling times
%   y: corresponding state values of a system at sampling times of t
%   u: input at sampling times of t
%   filename: string containing the filename where to save the graph, '' will not save anything
%OUTPUT:
%   none as of yet

%states vs time plot
subplot(2,1,1);
plot(t,y(:,1),'b',t,y(:,2),'r',t,y(:,3),'g',t,y(:,4),'k');
title('State variables variation');
legend('x [m]','dx [m/s]','theta [rad]','dtheta [rad/s]','input [N]');
xlabel('time[s]');
ylabel('value');

%Input vs time plot
subplot(2,1,2);
plot(t,u(:),'b');
title('Input');
legend('input [N]');
xlabel('time[s]');
ylabel('value');

if ~isempty(filename)
    print(filename,'-dpng');    %save graph to file as filename.png
end

end
