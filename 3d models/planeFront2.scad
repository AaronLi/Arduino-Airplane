difference(){
    union(){
        difference(){
            cube([91,76,12]);
            translate([3,3])cube([85,70,10]);
        }
        difference(){
            translate([9,9])cube([73,51,12]);
            translate([11.5,11.5])cube([68,46,10]);
            translate([9,40])cube([73,20,10]);
        }
        translate([0,0,12]){
            difference(){
                cube([91,76,11]);
                translate([3,5,2]){
                    cube([85,71,5]);
                    translate([8,13,5])cube([69,58,4]);
                }
            }
        }
    }
    translate([45.5,52,21])scale([1,1.5,1])rotate([0,0,90])import("propellerMount.stl");
    //translate([13,45])cube([30,20,23]);
    translate([0,73,17])rotate([0,90,0])cylinder(h=8,d=1.5,$fn=20);
    translate([87,73,17])rotate([0,90,0])cylinder(h=8,d=1.5,$fn=20);
    translate([11.5,11.5])cube([68,56,19]);
    translate([0,0,20])rotate([18,0,0])cube([91,20,20]);
    translate([0,0,20])rotate([0,-30])cube([20,76,20]);
    translate([82.325,0,25])rotate([0,30])cube([20,76,20]);
}
