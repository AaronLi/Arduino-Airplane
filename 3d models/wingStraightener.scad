difference(){
    translate([-35,0,-2])scale([1.9,1.9,1.9])union(){
        translate([18.9,0])cube([21.056,19,5]);
        translate([36.8,0,1.11])rotate([0,40])cube([21.056,19,5]);
    }
    cube([40,30,5]);
    translate([36.8,0,1.11])rotate([0,40])cube([40,30,5]);
}