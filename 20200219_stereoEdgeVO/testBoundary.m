function [pt_start_update, pt_end_update, n_search] = testBoundary(pt_start, pt_end, epiline, n_rows, n_cols)
persistent NW NE SW SE
if(isempty(NW))
   NW = [0,0]';
   NE = [n_cols,0]';
   SW = [0,n_rows]';
   SE = [n_cols,n_rows]';
end
% ������ ���ο� �����ϴ��� �Ǵ�.
% �� 2�� ����: �ƹ��͵� �� �ʿ� X
% �� 1�� ����: 1���� ���� Ž�� -> pt_end�� ������ ��� vs. ���� ���,
% �� 0�� ����: 0���� ���� or 2���� ���� ����.
% n_search : ��Ī ����.

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

% ���ο� �ִ� �� ����.
in_count = in_pt_start + in_pt_end;
if(in_count == 2) % �� ������.
   pt_start_update = pt_start;
   pt_end_update   = pt_end;
   n_search = round(norm(pt_end_update - pt_start_update));
elseif(in_count == 1) % pt_end�� �������� �ƴ��� �Ǵ��ض�.
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
   % �� ���� touch ���� ����. (���� �̿��ϸ��)
   if(s_nw*s_sw < 0) % ������ ��ġ ���� nw sw
      pt_update(1) = 1;
      pt_update(2) = epiline_section(2)/epiline_section(1)*(1-pt_end(1)) + pt_end(2);
   elseif(s_ne*s_se < 0) % ������ ��ġ ���� ne se
      pt_update(1) = n_cols;
      pt_update(2) = epiline_section(2)/epiline_section(1)*(n_cols-pt_end(1)) + pt_end(2);
   elseif(s_nw*s_ne < 0) % ��ܺ� ��ġ ���� nw ne
      pt_update(1) = epiline_section(1)/epiline_section(2)*(1-pt_end(2)) + pt_end(1);
      pt_update(2) = 1;
   elseif(s_sw*s_se < 0) % �ϴܺ� ��ġ ���� sw se
      pt_update(1) = epiline_section(1)/epiline_section(2)*(n_rows-pt_end(2)) + pt_end(1);
      pt_update(2) = n_rows;
   end
   if(in_pt_end > 0) % pt_end�� ������ ��� -> pt_start�� ������Ʈ
      pt_start_update = pt_update;
      pt_end_update   = pt_end;
   else % pt_end�� ���ΰ� �ƴ� ���. -> pt_end�� ������Ʈ
      pt_start_update = pt_start;
      pt_end_update   = pt_update;
   end
   n_search = round(norm(pt_end_update - pt_start_update));
else % �� 0�� ����.
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
   % �� ���� touch ���� ����. (���� �̿��ϸ��)
   cnt = 0;
   if(s_nw*s_sw < 0) % ������ ��ġ ���� nw sw
      cnt = cnt+1;
      pt_update(1,cnt) = 1;
      pt_update(2,cnt) = epiline_section(2)/epiline_section(1)*(1-pt_end(1)) + pt_end(2);
   end
   if(s_ne*s_se < 0) % ������ ��ġ ���� ne se
      cnt = cnt+1;
      pt_update(1,cnt) = n_cols;
      pt_update(2,cnt) = epiline_section(2)/epiline_section(1)*(n_cols-pt_end(1)) + pt_end(2);
   end
   if(s_nw*s_ne < 0) % ��ܺ� ��ġ ���� nw ne
      cnt = cnt+1;
      pt_update(1,cnt) = epiline_section(1)/epiline_section(2)*(1-pt_end(2)) + pt_end(1);
      pt_update(2,cnt) = 1;
   end
   if(s_sw*s_se < 0) % �ϴܺ� ��ġ ���� sw se
      cnt = cnt+1;
      pt_update(1,cnt) = epiline_section(1)/epiline_section(2)*(n_rows-pt_end(2)) + pt_end(1);
      pt_update(2,cnt) = n_rows;
   end
   
   if(cnt > 0) % 2��������.
      fprintf('cnt: %d\n',cnt);
      pt_start_update = pt_update(:,1);
      pt_end_update   = pt_update(:,2);
      % end-start ������ l_c�� ���� �����.
      if( dot(epiline,pt_end_update-pt_start_update) < 0 ) % l_c�� ���� ���� �����.
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