function anim(times,states,para,filename)
%INPUT:
%   times: discret timesteps where solution to state equations was computed
%   states: discret solutions to state equations at timesteps from times
%   para: physical parameter of problem
%   filename: filname to which save animation. use '' is save is not wanted
%OUTPUT:
%   none for the time being

figure(42)
%hold on
for i=1:size(times,2)
    a=[states(i,1),states(i,1) + (sin(states(i,3))*para.L)];    %basic trigonometry
    b=[0, cos(states(i,3))*para.L];
    plot(a,b);
    title('visual simulation')
    xlabel('xposition[m]');
    ylabel('yposition[m]');
    axis([-0.4 1 -0.1 0.1]);    %axis need some adjustements, on todo list
    %axis([-4 30 -4 4]);
    %axis([-1 12 -3 10]);
    %axis([-0.1 0.3 -0.1 0.3]);
    daspect([1 1 1]);   %to keep the correct aspect ratio (same scal axis)
    
    if filename ~= ''
        togif(filename,i)
    end
    pause(0.001);
end
%hold off
end

function togif(filename,framenum)   %writes animation to gif file
frame = getframe(42);
im = frame2im(frame);
[imind, cm] = rgb2ind(im,256);
if framenum == 1;
    imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'Delaytime',1/60); %60fps, not real speed
else
    imwrite(imind,cm,filename,'gif','WriteMode','append','Delaytime',1/60);
end
end