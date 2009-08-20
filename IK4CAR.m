%Lets make a paricle filter based Inverse kinematic solver.
clc;
clear all;
close all;

%%%%%%%%%%%%PROBLEM SPECIFIC%%%%%%%%%%%%%%
%set geometry
global a;
a =  0.6;
global b;
b = 0.4;
l = a+b;
global width;%Width of vehicle
width = .5;
global over;%Overshoot for wheel loc.
over = 0.05;
global wheellngth;%Length of wheel
wheellngth = 0.25;

%Initial task space co:ordinates
intl(1) = 2.3061;
intl(2) = 1.9080;
intl(3) = 1.1017;
%Desired task space co:ordinates
dsrd(1) = 2.8750;
dsrd(2) = 2.7500;

%set limits
stlinedist = errcal(dsrd,intl);
d_min = stlinedist;
d_max = stlinedist*pi;
psi_min = -(pi/3);
psi_max = (pi/3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%set std dev. of gaussian prob func.
stderr = 0.01;
%set spread while resampling
ressprd = 1;

%Lets populate
%Set number of particles
N = 100;
particles = zeros(2,N);
range = 10.0;
for pno = 1:N
    particles(1,pno) = rand*(range);
    particles(2,pno) = rand*(range);
end
wgt = zeros(N,1);

%The program is set to run only two iterations manually.
%Solution is obtained within that.
%If not set to iterate here using loop or apropriate conditons.
    
    %Clear graph
    cla;
    
    %Setting weights on each particle
    for pno = 1:N
        
        %modify to original parameters
        modfy(1) =  (particles(1,pno)*((d_max - d_min)/range)) + d_min;
        modfy(2) =  (particles(2,pno)*((psi_max - psi_min)/range)) + psi_min;
        
        %Calculate frwd kinematic of particle
        attnd = fwdkin(intl,modfy);
        
        %Calculate error of particle
        err = errcal(dsrd,[attnd(1),attnd(2)]);
        
        %Calculate weight of particle
        wgt(pno) = Gaussian(err,stderr);
        
    end
    
    %Plot particles
    axis ([0 range 0 range]);
    hold on;
    scatter(particles(1,:),particles(2,:),10,'filled','green');
    xlabel('d');
    ylabel('psi');
    input('wait');
    
    %Resampling
   while(sum(std(particles,0,2)) > 0.1)
        newparticles = zeros(2,N);
        newwgt = zeros(N,1);
        idx = randi(N);
        beta = 0.0;
        mw = max(wgt);
        
        for i = 1:N
            beta = beta + rand*2.0*mw;
            
            while (beta >= wgt(idx))
                beta = beta - wgt(idx);
                idx = mod(idx+1,N);
                if (idx == 0)
                    idx = 1;
                end
            end
            newparticles(:,i) = particles(:,idx);
            newwgt(i) = wgt(idx);
        end
        particles = newparticles;
        wgt = newwgt;
   end
    
    %Clear graph
    cla;
    
    %Plot particles
    scatter(newparticles(1,:),newparticles(2,:),10,'filled','red');
    xlabel('d');
    ylabel('psi');
    drawnow;
    sum(std(newparticles,0,2))
    input('wait');
    
    %Repopulate
   for pno = 1:N
       newparticles(1,pno) = newparticles(1,pno) + randn*ressprd;
       newparticles(2,pno) = newparticles(2,pno) + randn*ressprd;
   end

    particles = newparticles;
    
    %Plot particles
    scatter(particles(1,:),particles(2,:),10,'filled','green');
    xlabel('d');
    ylabel('psi');
    drawnow;
    sum(std(particles,0,2))
    input('wait');
    
    %Clear graph
    cla;
    
    %Setting weights on each particle
    for pno = 1:N
        
        %modify to original parameters
        modfy(1) =  (particles(1,pno)*((d_max - d_min)/range)) + d_min;
        modfy(2) =  (particles(2,pno)*((psi_max - psi_min)/range)) + psi_min;
        
        %Calculate frwd kinematic of particle
        attnd = fwdkin(intl,modfy);
        
        %Calculate error of particle
        err = errcal(dsrd,attnd);
        
        %Calculate weight of particle
        wgt(pno) = Gaussian(err,stderr);   
        
    end
    
    %Resampling
    while( sum(std(particles,0,2)) > 0.1)
        newparticles = zeros(2,N);
        newwgt = zeros(N,1);
        idx = randi(N);
        beta = 0.0;
        mw = max(wgt);
    
        for i = 1:N
            beta = beta + rand*2.0*mw;
        
            while (beta >= wgt(idx))
                beta = beta - wgt(idx);
                idx = mod(idx+1,N);
                if (idx == 0)
                    idx = 1;
                end
            end
        
            newparticles(:,i) = particles(:,idx);
            newwgt(i) = wgt(idx);
        end
    
        particles = newparticles;
        wgt = newwgt;
    end
    
    %Plot particles
    scatter(particles(1,:),particles(2,:),10,'filled','blue');
    xlabel('d');
    ylabel('psi');
    drawnow;
    sum(std(particles,0,2))
    input('wait');
   
%end of looping.

sol(1) = (particles(1,pno)*((d_max - d_min)/range)) + d_min;
sol(2) = (particles(2,pno)*((psi_max - psi_min)/range)) + psi_min;
sol