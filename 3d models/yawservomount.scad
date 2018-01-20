//beginning segment
translate([17.5,2.5,4])cube([25,2,16],center=true);
translate([17.5,-2.5,4])cube([25,2,16],center=true);
translate([17.5,0,-3])cube([25,7,2],center=true);

//middle segment
translate([0,2.5,6])cube([10,2,12],center=true);
translate([0,-2.5,6])cube([10,2,12],center=true);
translate([0,0,0])cube([10,7,2],center=true);


//end segment
difference(){
    translate([-10,0,3])cube([10,7,2],center=true);//yaw servo screw is 5mm above base of yaw: thickness 2 so shift 5-2 up
    translate([-12,0,2])cylinder(h=2, d=3, $fn=7);
}
//walls of end segment
translate([-10,-2.5,7.5])cube([10,2,9],center=true);
translate([-10,2.5,7.5])cube([10,2,9],center=true);
