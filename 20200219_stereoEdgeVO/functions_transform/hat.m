function res = hat( vec )
%HAT �� �Լ��� ��� ���� ��ġ
%   �ڼ��� ���� ��ġ
if(length(vec)~=3)
   assert(false,'function hat : vector must be 3-dims.');
end
res = [0,-vec(3),vec(2);...
       vec(3), 0, -vec(1);...
       -vec(2),vec(1),0];

end

