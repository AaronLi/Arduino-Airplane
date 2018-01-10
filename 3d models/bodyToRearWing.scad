difference(){
    translate([-35,0,-2])scale([1.9,1.9,1.9])union(){
        translate([18.9,0])cube([21.056,42.1052632,7]);
        translate([38.7,0,0.1])rotate([0,10])cube([21.056,42.1052632,7]);
    }
    translate([0,0,1.65])cube([40,80,5]);
    //translate([36.8,0,1.11])rotate([0,30])cube([40,30,5]);
}