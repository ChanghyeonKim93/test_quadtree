close all;clear all;clc;
%%
len = 1000;
d_truth = 0.4;

a_init = 0.5;
b_init = 0.5;
mu_init = 0;
sig_init = 5;
z_min = 0.03;
z_max = 10;

a_save = zeros(1,len);
b_save = zeros(1,len);
mu_save = zeros(1,len);
sig_save = zeros(1,len);


tau_save = abs(randn(1,len)*0.01) + 0.04;
rho_truth   = 0.7;
len_inlier  = floor(rho_truth*len);
len_outlier = len - len_inlier;

d_measure = [randn(1,len_inlier).*0.01 + d_truth,1*(rand(1,len_outlier)-0.5)+d_truth];
d_measure = d_measure(1,randperm(length(d_measure)));

a_save(1)   = a_init;
b_save(1)   = b_init;
mu_save(1)  = d_measure(1);
sig_save(1) = tau_save(1);


mu_save2 = zeros(1,len);
sig_save2 = zeros(1,len);

mu_save2(1) = d_measure(1);
sig_save2(1) = tau_save(1);

for i = 2:len
   a_prev = a_save(i-1);
   b_prev = b_save(i-1);
   sig_prev = sig_save(i-1);
   mu_prev = mu_save(i-1);
   
   if(z_min > d_measure(i))
      z_min = d_measure(i);
   end
   if(z_max < d_measure(i))
      z_max = d_measure(i);
   end
   
   m = (sig_prev^2*d_measure(i)+tau_save(i)^2*mu_prev)/(sig_prev^2+tau_save(i)^2);
   s = sqrt(tau_save(i)^2*sig_prev^2/(sig_prev^2+tau_save(i)^2));
   C1 = 1/sqrt(2*pi*(sig_prev^2+tau_save(i)^2))*exp(-(d_measure(i)-mu_prev)^2/(tau_save(i)^2+sig_prev^2)*0.5);
   C2 = b_prev/(a_prev+b_prev)*1/(z_max-z_min);
   %    C1C2 = sqrt(C1^2+C2^2);
   C1C2 = C1 + C2;
   C1   = C1 / C1C2;
   C2   = C2 / C1C2;
   
   A = C1*(a_prev+1)/(a_prev+b_prev+1)+C2*a_prev/(a_prev+b_prev+1);
   B = 1/A*(C1*(a_prev+1)*(a_prev+2)/(a_prev+b_prev+1)/(a_prev+b_prev+2) + C2*(a_prev+1)*a_prev/(a_prev+b_prev+1)/(a_prev+b_prev+2));
   [a_save(i),b_save(i),mu_save(i),sig_save(i),z_min, z_max] =...
      updateDF(d_measure(i), tau_save(i), a_prev, b_prev, mu_prev, sig_prev, z_min, z_max);
   
   % 그냥 추정
   mu_save2(i) = (sig_save2(i-1)^2*d_measure(i)+tau_save(i)^2*mu_save2(i-1))/(sig_save2(i-1)^2+tau_save(i)^2);
   sig_save2(i) = sqrt(tau_save(i)^2*sig_save2(i-1)^2/(sig_save2(i-1)^2+tau_save(i)^2));
end

figure();
plot(mu_save,'k','linewidth',2);hold on;
plot(mu_save-2*sig_save,'m--');
plot(mu_save+2*sig_save,'m--');
plot(d_measure,'c');
ylim([-1,1]*0.3+d_truth);

figure();
plot(a_save./(a_save+b_save));


figure();
plot([0,length(mu_save)],[1,1]*d_truth,'r--','linewidth',1.5);hold on;
plot(mu_save2,'k','linewidth',1);
plot(mu_save,'m');
ylim([-1,1]*0.3+d_truth);
