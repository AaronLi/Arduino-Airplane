difference(){
    translate([-35,0,-2])scale([1.9,1.9,1.9])union(){
        translate([18.45,0])cube([16.2,42.1052632,7]);
        translate([31.19,0,0.94])rotate([0,30])cube([21.056,42.1052632,7]);
    }
    translate([0,0,1.65])cube([30,80,5]);
    translate([27.2,80,-1.95])rotate([180,10])cube([50,80,20]);
}