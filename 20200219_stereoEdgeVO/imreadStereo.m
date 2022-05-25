function [img_l,img_r] = imreadStereo(data_info,n_img, flag_histeq)

img_l = imread(data_info.leftnames{n_img});
img_r = imread(data_info.rightnames{n_img});

if(size(img_l,3) > 1)
   img_l = double(rgb2gray(img_l));
else
   img_l = double(img_l);
end

if(size(img_r,3) > 1)
   img_r = double(rgb2gray(img_r));
else
   img_r = double(img_r);
end

if(flag_histeq == true)
%    img_l = double(cv.CLAHE(img_l,'ClipLimit',2,'TileGridSize',[22,22]));
%    img_r = double(cv.CLAHE(img_r,'ClipLimit',2,'TileGridSize',[22,22]));
   % img_l = imgaussfilt(img_l,1);
   % img_r = imgaussfilt(img_r,1);
   
   % img_l = double(uint8(2*(img_l-10)+20));
   % img_r = double(uint8(2*(img_r-10)+20));
   
   img_l = double(uint8(1.5*(img_l-10)+10));
   img_r = double(uint8(1.5*(img_r-10)+10));
end

end