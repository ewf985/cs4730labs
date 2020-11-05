 %Ready position
 XYZ=[245.0773,-177.6170,-66.4708];
 OAT=[0.7192,179.9859,-106.1681];
 
 %PUMA values
 al=[-90,0,90,-90,90,0];
 rat=unitsratio('mm','inch');
 d=[0,rat*4.9375,0,rat*8,0,rat*2.202];
 a=[0,rat*8,1,0,0,0];
 
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
 
 %New variables for ease of implementation
 px=XYZ(1);
 py=XYZ(2);
 pz=XYZ(3);
 
 %Initialization of T0>6
 T=[nx ny nz px;
    ox oy oz py;
    ax ay az pz;
    0  0  0  1;]
 
 %Placeholders because implementing these functions is hellish
 top=0;
 bot=0;

 %Calculations for each theta in t()
 t(1)=atan2d(py,px)
 t234=atan2d(az,(cosd(t(1))*ax)+(sind(t(1))*ay))
 
 top=
 t(3)=acosd(power((px*cosd(t(1)))+(py*sind(t(1))-(cosd(t234)*a(4))),2)+(pz-power(sind(t234)*a(4),2))-power(a(2),2)-power(a(3),2))./(2*a(2)*a(3))
 
 top=((cosd(t(3))*a(3))+a(2))*((pz-(sind(t234)*a(4)))*((px*cosd(t(1)))+(py*sind(t(1)))-(cosd(t234)*a(4))))
 bot=(((cosd(t(3))*a(3))+a(2))*((px*cosd(t(1)))+(py*sind(t(1))-(cosd(t234)*a(4)))))+((sind(t(3))*a(3))*(pz-(sind(t234)*a(4))))
 
 t(2)=atan2d(top,bot);