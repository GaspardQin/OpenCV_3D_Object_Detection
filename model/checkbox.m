I = checkerboard(150,5,5);

K = (I > 0.5); 
%imshow(I);
imwrite(K,'checkerboard.bmp');