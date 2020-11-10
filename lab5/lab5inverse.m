%  %Ready position
%  XYZ=[245.0773,-177.6170,-66.4708]
%  OAT=[0.7192,179.9859,-106.1681]

%Nest position
XYZ=[66.3492,-180.3498,-243.1065]
OAT=[0.8098,179.8571,-166.1720]


% %All 30
% XYZ=[281.9175,323.7249,3.2447]
% OAT=[134.5025,-3.3258,55.7053]

 %PUMA values
 al=[-90,0,90,-90,90,0];
 rat=unitsratio('mm','inch');
 d=[0,rat*4.9375,0,rat*8,0,rat*2.202];
 a=[0,rat*8,0,0,0,0];
 
 t=zeros(1,6);
 
 %Conversion from OAT to rotation matrix
 nx=(-sind(OAT(1))*sind(OAT(2))*cosd(OAT(3)))+(cosd(OAT(1))*sind(OAT(3)));
 ny=(sind(OAT(1))*sind(OAT(2))*sind(OAT(3)))+(cosd(OAT(1))*cosd(OAT(3)));
 nz=sind(OAT(1))*cosd(OAT(2));
 ox=(cosd(OAT(1))*sind(OAT(2))*cosd(OAT(3)))+(sind(OAT(1))*sind(OAT(3)));
 oy=(-cosd(OAT(1))*sind(OAT(2))*sind(OAT(3)))+(sind(OAT(1))*cosd(OAT(3)));
 oz=-cosd(OAT(1))*cosd(OAT(2));
 ax=-cosd(OAT(2))*cosd(OAT(3));
 ay=cosd(OAT(2))*sind(OAT(3));
 az=-sind(OAT(2));
 
 %Initialization of T0>6
 T=[nx ny nz XYZ(1);
    ox oy oz XYZ(2);
    ax ay az XYZ(3);
    0  0  0  1;];
 
 %Calculation of P4>0
 p64=[nz*d(6);oz*d(6);az*d(6)];
 p40=[XYZ(1)-p64(1);XYZ(2)-p64(2);XYZ(3)-p64(3)];
 px=p40(1);
 py=p40(2);
 pz=p40(3);
 
 %Calculation of t(1)
 top=(py*sqrt(power(px,2)+power(py,2)-power(d(2),2)))-(d(2)*px);
 bot=(px*sqrt(power(px,2)+power(py,2)-power(d(2),2)))+(d(2)*py);
 t(1)=atan2d(top,bot);
 
 %Calculation of t(3)
 r=2*a(2)*sqrt(power(a(3),2)+power(d(4),2));
 R=power(px,2)+power(py,2)+power(pz,2)-power(a(2),2)-power(a(3),3)-power(d(2),2)-power(d(4),2);
 
 top=(2*a(2)*d(4)*R)-(2*a(2)*a(3)*sqrt(power(r,2)-power(R,2)));
 bot=(2*a(2)*d(4)*sqrt(power(r,2)-power(R,2)))+(2*a(2)*a(3)*R);
 t(3)=atan2d(top,bot);
 
 %Calculation of t(2)
 top=(pz*(a(2)+(a(3)*cosd(t(3)))+(d(4)*sind(t(3)))))+(((d(4)*cosd(t(3)))-(a(3)*sind(t(3))))*sqrt(power(px,2)+power(py,2)-power(d(2),2)));
 bot=(pz*((d(4)*cosd(t(3)))-(a(3)*sind(t(3)))))-((a(2)+(a(3)*cosd(t(3)))+(d(4)*sind(t(3))))*sqrt(power(px,2)+power(py,2)-power(d(2),2)));
 t(2)=atan2d(top,-bot);
 
 %Calculation of T1, T2
 T1=[cosd(t(1))*cosd(t(2)+t(3)), -sind(t(1)), cosd(t(1))*sind(t(2)+t(3)), (a(2)*cosd(t(1))*cosd(t(2)))+(a(3)*cosd(t(1))*cosd(t(2)+t(3)))-(d(2)*sind(t(1)));
     sind(t(1))*cosd(t(2)+t(3)), cosd(t(1)),  sind(t(1))*sind(t(2)+t(3)), (a(2)*sind(t(1))*cosd(t(2)))+(a(3)*sind(t(1))*cosd(t(2)+t(3)))-(d(2)*cosd(t(1)));
     -sind(t(2)+t(3)),           0,           cosd(t(2)+t(3)),            (-a(2)*sind(t(2)))-(a(3)*sind(t(2)+t(3)));
     0                           0            0                           1];
 
 T2=inv(T1)*T;
 
 %Calculation of t(4), t(5), t(6)
 t(4)=atan2d(T2(2,3),T2(1,3));
 t(6)=atan2d(T2(3,2),-T2(3,1));
 t(5)=atan2d(T2(3,2),T2(3,3)*sind(t(6)))
 
 