difference(){
    union(){
        cube([32,38,2],center=true);
        //translate([0,0,1])cylinder(h=100, d=27.7); //uncomment for motor analogue
    }
    translate([9.2,0,-1])cylinder(h=2, d=4,$fn=7);
    translate([-9.2,0,-1])cylinder(h=2, d=4, $fn=7);
    translate([0,7.7,-1])cylinder(h=2, d=4,$fn=7);
    translate([0,-7.7,-1])cylinder(h=2, d=4, $fn=7);
    translate([0,0,-1])cylinder(h=2, d=8.6, $fn=20);
}
translate([0,18,-7.7])cube([32,12,2],center=true);
translate([0,-18,-7.7])cube([32,12,2],center=true);
translate([0,-13,-4])cube([32,2,6],center=true);
translate([0,13,-4])cube([32,2,6],center=true);