$fn=40;

width = 45;
length = 94;

difference(){
    hull(){
        translate([0,0,0]){cylinder(d1=9,d2=12,h=2);translate([0,0,2])cylinder(d1=12,d2=10,h=6);}
        translate([0,width,0]){cylinder(d1=9,d2=12,h=2);translate([0,0,2])cylinder(d1=12,d2=10,h=6);}
        translate([length,width,0]){cylinder(d1=9,d2=12,h=2);translate([0,0,2])cylinder(d1=12,d2=10,h=6);}
        translate([length,0,0]){cylinder(d1=9,d2=12,h=2);translate([0,0,2])cylinder(d1=12,d2=10,h=6);}
    }
    hull(){
        translate([0,0,2])cylinder(d=9,h=7);
        translate([0,width,2])cylinder(d=9,h=7);
        translate([length,width,2])cylinder(d=9,h=7);
        translate([length,0,2])cylinder(d=9,h=7);
            
    }
        translate([0,0,-0.01])cylinder(d1=5.5,d2=3.2,h=2.1);
        translate([0,width,-0.01])cylinder(d1=5.5,d2=3.2,h=2.1);
        translate([length,width,-0.01])cylinder(d1=5.5,d2=3.2,h=2.1);
        translate([length,0,-0.01])cylinder(d1=5.5,d2=3.2,h=2.1);
    
    translate([20-4,28.5,-0.01])cube([8,8,8]);
     translate([20+56,12.5,-0.01])cylinder(d=3,h=10);
    translate([20+56,32.5,-0.01])cylinder(d=3,h=10);
}