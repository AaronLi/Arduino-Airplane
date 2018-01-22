

//middle segment
translate([10,3.5,10.25])cube([30,3,23.5],center=true);
translate([10,-3.5,10.25])cube([30,3,23.5],center=true);// x translate=1/2cubelength-5
translate([10,0,0])cube([30,7,3],center=true);


//attach | end segment
difference(){
    translate([-10,0,3])cube([10,7,3],center=true);//yaw servo screw is 5mm above base of yaw: thickness 2 so shift 5-2 up
    translate([-12,0,1])cylinder(h=4, d=3, $fn=7);
}
//walls of end segment
translate([-10,-3.5,7.5+5-0.75])cube([10,3,9+11.5],center=true);
translate([-10,3.5,11.75])cube([10,3,20.5],center=true);
