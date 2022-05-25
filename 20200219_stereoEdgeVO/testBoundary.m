function [pt_start_update, pt_end_update, n_search] = testBoundary(pt_start, pt_end, epiline, n_rows, n_cols)
persistent NW NE SW SE
if(isempty(NW))
   NW = [0,0]';
   NE = [n_cols,0]';
   SW = [0,n_rows]';
   SE = [n_cols,n_rows]';
end
% 점들이 내부에 존재하는지 판단.
% 점 2개 내부: 아무것도 할 필요 X
% 점 1개 내부: 1개의 절편 탐색 -> pt_end가 내부인 경우 vs. 밖인 경우,
% 점 0개 내부: 0개의 절편 or 2개의 절편 존재.
% n_search : 서칭 길이.

in_pt_start = false;
in_pt_end   = false;
if(pt_start(1) > 0 && pt_start(1) < n_cols && pt_start(2) > 0 && pt_start(2) < n_rows)
   in_pt_start = true;
else
   in_pt_start = false;
end
if(pt_end(1) > 0 && pt_end(1) < n_cols && pt_end(2) > 0 && pt_end(2) < n_rows)
   in_pt_end = true;
else
   in_pt_end = false;
end

% 내부에 있는 점 갯수.
in_count = in_pt_start + in_pt_end;
if(in_count == 2) % 걍 나가라.
   pt_start_update = pt_start;
   pt_end_update   = pt_end;
   n_search = round(norm(pt_end_update - pt_start_update));
elseif(in_count == 1) % pt_end가 내부인지 아닌지 판단해라.
   epiline_section = pt_end - pt_start;
   
   vec_nw  = NW - pt_start;
   vec_sw  = SW - pt_start;
   vec_se  = SE - pt_start;
   vec_ne  = NE - pt_start;
   
   s_nw = cross2d(epiline_section, vec_nw);
   s_sw = cross2d(epiline_section, vec_sw);
   s_se = cross2d(epiline_section, vec_se);
   s_ne = cross2d(epiline_section, vec_ne);
   
   pt_update = zeros(2,1);
   % u = lu/lv*(v-pv)+pu or v = lv/lu*(u-pu)+pv;
   % 네 변의 touch 여부 결정. (외적 이용하면댐)
   if(s_nw*s_sw < 0) % 좌측변 터치 여부 nw sw
      pt_update(1) = 1;
      pt_update(2) = epiline_section(2)/epiline_section(1)*(1-pt_end(1)) + pt_end(2);
   elseif(s_ne*s_se < 0) % 우측변 터치 여부 ne se
      pt_update(1) = n_cols;
      pt_update(2) = epiline_section(2)/epiline_section(1)*(n_cols-pt_end(1)) + pt_end(2);
   elseif(s_nw*s_ne < 0) % 상단변 터치 여부 nw ne
      pt_update(1) = epiline_section(1)/epiline_section(2)*(1-pt_end(2)) + pt_end(1);
      pt_update(2) = 1;
   elseif(s_sw*s_se < 0) % 하단변 터치 여부 sw se
      pt_update(1) = epiline_section(1)/epiline_section(2)*(n_rows-pt_end(2)) + pt_end(1);
      pt_update(2) = n_rows;
   end
   if(in_pt_end > 0) % pt_end가 내부인 경우 -> pt_start를 업데이트
      pt_start_update = pt_update;
      pt_end_update   = pt_end;
   else % pt_end가 내부가 아닌 경우. -> pt_end가 업데이트
      pt_start_update = pt_start;
      pt_end_update   = pt_update;
   end
   n_search = round(norm(pt_end_update - pt_start_update));
else % 점 0개 내부.
   epiline_section = pt_end - pt_start;
   
   vec_nw  = NW - pt_start;
   vec_sw  = SW - pt_start;
   vec_se  = SE - pt_start;
   vec_ne  = NE - pt_start;
   
   s_nw = cross2d(epiline_section, vec_nw);
   s_sw = cross2d(epiline_section, vec_sw);
   s_se = cross2d(epiline_section, vec_se);
   s_ne = cross2d(epiline_section, vec_ne);
   pt_update = zeros(2,2);
   % u = lu/lv*(v-pv)+pu or v = lv/lu*(u-pu)+pv;
   % 네 변의 touch 여부 결정. (외적 이용하면댐)
   cnt = 0;
   if(s_nw*s_sw < 0) % 좌측변 터치 여부 nw sw
      cnt = cnt+1;
      pt_update(1,cnt) = 1;
      pt_update(2,cnt) = epiline_section(2)/epiline_section(1)*(1-pt_end(1)) + pt_end(2);
   end
   if(s_ne*s_se < 0) % 우측변 터치 여부 ne se
      cnt = cnt+1;
      pt_update(1,cnt) = n_cols;
      pt_update(2,cnt) = epiline_section(2)/epiline_section(1)*(n_cols-pt_end(1)) + pt_end(2);
   end
   if(s_nw*s_ne < 0) % 상단변 터치 여부 nw ne
      cnt = cnt+1;
      pt_update(1,cnt) = epiline_section(1)/epiline_section(2)*(1-pt_end(2)) + pt_end(1);
      pt_update(2,cnt) = 1;
   end
   if(s_sw*s_se < 0) % 하단변 터치 여부 sw se
      cnt = cnt+1;
      pt_update(1,cnt) = epiline_section(1)/epiline_section(2)*(n_rows-pt_end(2)) + pt_end(1);
      pt_update(2,cnt) = n_rows;
   end
   
   if(cnt > 0) % 2개여야함.
      fprintf('cnt: %d\n',cnt);
      pt_start_update = pt_update(:,1);
      pt_end_update   = pt_update(:,2);
      % end-start 방향이 l_c와 같게 만든다.
      if( dot(epiline,pt_end_update-pt_start_update) < 0 ) % l_c와 방향 같게 만들어.
         pt_temp = pt_start_update;
         pt_start_update = pt_end_update;
         pt_end_update = pt_temp;
      end
      n_search = round(norm(pt_end_update - pt_start_update));
   else
      pt_start_update = zeros(2,1);
      pt_end_update   = zeros(2,1);
      n_search = 0;
   end
end
end