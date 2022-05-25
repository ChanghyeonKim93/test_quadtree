function [a,b,mu,sig,z_min, z_max] = updateDF(invd, std_invd, a_prev, b_prev, mu_prev, sig_prev, z_min_prev, z_max_prev)

if(z_min_prev > invd)
   z_min = invd;
else
   z_min = z_min_prev;
end
if(z_max_prev < invd)
   z_max = invd;
else
   z_max = z_max_prev;
end

m = (sig_prev^2*invd+std_invd^2*mu_prev)/(sig_prev^2+std_invd^2);
s = sqrt(std_invd^2*sig_prev^2/(sig_prev^2+std_invd^2));
C1 = 1/sqrt(2*pi*(sig_prev^2+std_invd^2))*exp(-(invd-mu_prev)^2/(std_invd^2+sig_prev^2)*0.5);
C2 = b_prev/(a_prev+b_prev)*1/(z_max-z_min);
C1C2 = C1 + C2;
C1   = C1 / C1C2;
C2   = C2 / C1C2;

A = C1*(a_prev+1)/(a_prev+b_prev+1)+C2*a_prev/(a_prev+b_prev+1);
B = 1/A*(C1*(a_prev+1)*(a_prev+2)/(a_prev+b_prev+1)/(a_prev+b_prev+2) + C2*(a_prev+1)*a_prev/(a_prev+b_prev+1)/(a_prev+b_prev+2));

a   = A/(A-B)*(B-1);
b   = 1/(B-A)*(A-1)*(B-1);
mu  = C1*m + C2*mu_prev;
sig = sqrt(C1*(m^2+s^2)+C2*(mu_prev^2+sig_prev^2)-mu^2);
end