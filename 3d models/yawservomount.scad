//beginning segment
translate([17.5,3.5,4])cube([25,3,16],center=true);
translate([17.5,-3.5,4])cube([25,3,16],center=true);
translate([17.5,0,-3])cube([25,7,3],center=true);

//middle segment
translate([0,3.5,6])cube([10,3,12],center=true);
translate([0,-3.5,6])cube([10,3,12],center=true);
translate([0,0,0])cube([10,7,3],center=true);


//end segment
difference(){
    translate([-10,0,3])cube([10,7,3],center=true);//yaw servo screw is 5mm above base of yaw: thickness 2 so shift 5-2 up
    translate([-12,0,1])cylinder(h=4, d=3, $fn=7);
}
//walls of end segment
translate([-10,-3.5,7.5])cube([10,3,9],center=true);
translate([-10,3.5,7.5])cube([10,3,9],center=true);
