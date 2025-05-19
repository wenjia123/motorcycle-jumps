clear all;
close all;
clc;

%measurement 
b=57;  %bike base length(in)
h_lift=13.25; %lifted height(in)
c=1.5; % in
lambda=1.1; % in radians 
xtt=39;
lff=33.5;
xff=xtt+(lff/2)*cos(lambda)
rfw=14; %radius of front wheel
rrw=13; %radius of rear wheel
mff=10; %guess kg
mfw=10; %guess kg
mrw=13; %guess kg
hff=(lff/2)*sin(lambda)+rfw
mtotal=140;
mrf=mtotal-mff-mfw-mrw;
mtotal_withrider=191.1;
mrf_rider=mtotal_withrider-mff-mfw-mrw;

% bike on the flat 
mf= 68.2;
mr= 71.8; 

% with ridder on the bike
mf_rider=93.4;
mr_rider=97.7;

% with the bike lifted angle theta 
mr_lift= 76.3;
mf_lift=mtotal-mr_lift;

%bike lifted angle theta and with rider sitting on it
mr_lift_rider=123.9;
mf_lift_rider=mtotal_withrider-mr_lift_rider;


% cacluation 
theta = asind(h_lift/b);
atotal = (mf*b)/mtotal;
xrf = (atotal*mtotal-xff*mff-mfw*b)/mr;
htotal= cot(asind(h_lift/b))*((mr_lift*b)/mtotal-(b-atotal))+(rfw+rrw)/2;
hrf=(htotal*mtotal-mfw*rfw-mff*hff-mrw*rrw)/mrf;

% with rider
atotal_rider = (mf_rider*b)/mtotal_withrider;
xrf_rider = (atotal_rider*mtotal_withrider-xff*mff-mfw*b)/mr_rider;
htotal_rider= cot(asind(h_lift/b))*((mr_lift_rider*b)/mtotal_withrider-(b-atotal_rider))+(rfw+rrw)/2;
hrf_rider=(htotal_rider*mtotal_withrider-mfw*rfw-mff*hff-mrw*rrw)/mrf_rider;



atotal
xrf
htotal 
hrf

atotal_rider
xrf_rider
htotal_rider
hrf_rider
