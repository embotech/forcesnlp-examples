clear variables
close all 
clc

hover_omega = 40;
init_guess = hover_omega*ones(1,4).';

ss_input = fsolve(@(input) dynamics_ss(input), init_guess)