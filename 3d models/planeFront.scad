difference(){
    union(){
        translate([0,0,12]){
            difference(){
                cube([86,66,11]);
                translate([8,0,3])cube([70,50,5.2]);
                translate([18,0,6]){cube([50,40,5]);
                scale([1, 1.21,1.1])translate([25,16,4])rotate([0,0,90])import("C:\\Users\\dumpl\\Documents\\OpenSCAD\\propellerMount.stl");
                }
                translate([0,2.5,5])rotate([0,90,0])cylinder(d=1.35,h=8,$fn=20);
                //translate([0,2.5,4.25])cube([1.5,30,1.5]);
                translate([0.1,32.5,5])rotate([90,0,0])cylinder(h=30,d=1.35,$fn=20);
                translate([0,20,5])rotate([0,90,0])cylinder(d=8,h=3,$fn=20);
                translate([78,2.5,5])rotate([0,90,0])cylinder(d=1.35,h=8,$fn=20);
                //translate([84.5,2.5,4.25])cube([1.5,30,1.5]);
                translate([85.9,32.5,5])rotate([90,0,0])cylinder(h=30,d=1.35,$fn=20);
                translate([83,20,5])rotate([0,90,0])cylinder(d=8,h=3,$fn=20);
            }
        }
        union(){
            difference(){
                translate([8,8])cube([70,30,10]);
                translate([13,13])cube([60,40,10]);
            }
            difference(){
                cube([86,66,12]);
                translate([3,3])cube([80,60,10]);
            }
        }
    }
    translate([19,5,0])cube([20,10,21]);
    translate([-2,0,19])rotate([0,-30,0])cube([20,70,20]);
    translate([76,0,25.2])rotate([0,30,0])cube([20,70,20]);
    translate([0,40,30])rotate([-30,0,0])cube([86,70,20]);
}