 %t=[30,30,30,30,30,30]; %all 30
 %t=[-179.124,-143.553,-13.559,90.139,90.074,50.944]; %ready position
 %t=[-179.124,-203.557,-13.559,90.139,90.074,50.944]; %nest position
 %t=[-170.479,-146.691,-10.469,90.925,74.079,66.301]; %arbitrary position
 %t=[-65.2472   25.6999   14.7734  149.6153  126.3708   54.6654]; %Alternate ready position
 t=[-104.8393  -11.3878   14.7734 -164.3226   93.4028   14.7424]; %Alternate nest position
 
 al=[-90,0,90,-90,90,0];
 rat=unitsratio('mm','inch');
 d=[0,rat*4.9375,0,rat*8,0,rat*2.202];
 a=[0,rat*8,0,0,0,0];
 A=zeros(4,4,6);
 for i=1:6
     A(:,:,i)=[ cosd(t(i)),    -sind(t(i))*cosd(al(i)),  sind(t(i))*sind(al(i)),   a(i)*cosd(t(i));
            sind(t(i)),    cosd(t(i))*cosd(al(i)),     -cosd(t(i))*sind(al(i)),  a(i)*sind(t(i));
            0,             sind(al(i)),              cosd(al(i)),              d(i);
            0,             0,                        0,                        1];
 end
 T=A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4)*A(:,:,5)*A(:,:,6);
 
 XYZ=zeros(1,3);
 XYZ(1)=T(1,4);
 XYZ(2)=T(2,4);
 XYZ(3)=T(3,4);
 XYZ
 
 rotm=[T(1,1) T(1,2) T(1,3);
       T(2,1) T(2,2) T(2,3);
       T(3,1) T(3,2) T(3,3)];
 OAT=zeros(1,3);
 OAT(1)=atan2d(rotm(1,3),-rotm(2,3));
 OAT(2)=atan2d(cosd(OAT(1))*rotm(3,3),rotm(2,3));
 OAT(3)=atan2d(rotm(3,2),-rotm(3,1));
 OAT