% Written by Chuong Nguyen Oct 20, 2020
% For 2D jumping
% s1=0; 
% The sideSign does not effect the x, z direction since l1=0

function [J,p]=computeLegJacobianAndPosition_2D(q)
 
    l1=0.2; % thigh length
    l2=0.2; % calf length

    s1=sin(q(1)); % for thigh joint
    s2=sin(q(2)); % for calf joint
    
    c1=cos(q(1)); % for thigh joint
    c2=cos(q(2)); % for calf joint
    
    c12=c1*c2-s1*s2;
    s12=s1*c2+c1*s2;
    
    J(1,1)=-l2*c12-l1*c1;
    J(2,1)=l1*s1+l2*s12;
    
    J(1,2)=-l2*c12;
    J(2,2)=l2*s12;
    
    p(1)=-l2*s12-l1*s1; % in x direction
    p(2)=-l2*c12-l1*c1; % in z direction
    
 end