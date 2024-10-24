% Written by Chuong Nguyen Oct 20, 2020
% For 2D jumping
% s1=0; 
% The sideSign does not effect the x, z direction since l1=0

function [J,p]=computeLegJacobianAndPosition(q,leg)
 
    l1=0.083; % hip length
    l2=0.2; % thigh length
    l3=0.2; % calf length
    
    if leg==1 || leg==3 % left leg has sideSign 1
        sideSign=1;
    else
        sideSign=-1; % right leg has sideSign -1
    end
    s1=sin(q(1)); % for hip joint
    s2=sin(q(2)); % for thigh joint
    s3=sin(q(3)); % for calf joint
    
    c1=cos(q(1)); % for hip joint
    c2=cos(q(2)); % for thigh joint
    c3=cos(q(3)); % for calf joint
    
    c23=c2*c3-s2*s3;
    s23=s2*c3+c2*s3;
    s1=0; % for 2D jumping
    
    J(1,1)=0;
    J(2,1)=-sideSign*l1*s1+l2*c2*c1+l3*c23*c1;
    J(3,1)=sideSign*l1*c1+l2*c2*s1+l3*c23*s1;
    
    J(1,2)=-l3*c23-l2*c2;
    J(2,2)=-l2*s2*s1-l3*s23*s1;
    J(3,2)=l2*s2*c1+l3*s23*c1;
    
    J(1,3)=-l3*c23;
    J(2,3)=-l3*s23*s1;
    J(3,3)=l3*s23*c1;
    
    p(1)=-l3*s23-l2*s2; % in x direction
    p(2)=l1*sideSign*c1+l3*(s1*c23)+l2*c2*s1; % in y direction
    p(3)=l1*sideSign*s1-l3*(c1*c23)-l2*c1*c2; % in z direction
    
 end