function res = hat( vec )
%HAT 이 함수의 요약 설명 위치
%   자세한 설명 위치
if(length(vec)~=3)
   assert(false,'function hat : vector must be 3-dims.');
end
res = [0,-vec(3),vec(2);...
       vec(3), 0, -vec(1);...
       -vec(2),vec(1),0];

end

