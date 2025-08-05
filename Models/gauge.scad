use <./keycap_box.scad>;

// Parameters
base_diameter = 58.3;
base_height   = 10;

top_diameter  = 52.25;
top_height    = 15; // additional height on top of base

fillet_radius = 1;  // size of rounded bottom edge
wall_thickness = 2.0;

screen_length = 31;
screen_height = 11.85;

$fn = 170;

// Rounded base using Minkowski sum
module base_disk_rounded() {
    minkowski() {
        cylinder(h = base_height - fillet_radius, d = base_diameter-(fillet_radius*2));
        sphere(r = fillet_radius, $fn = 50);
    }
}

// Top disk on top
module top_disk() {
    translate([0, 0, base_height])
        cylinder(h = top_height, d = top_diameter);
}

difference() {
    union() {          
        base_disk_rounded();
        top_disk();
    }
    translate([0, 0, wall_thickness])
    cylinder(
        h = top_height+base_height,
        d = top_diameter - 2 * wall_thickness,
        $fn = 60
    );
    translate([8.5,8,4.9])
    rotate([180,0,0])
    keycap_hollow();
    translate([-1,0,0])
    rotate([0,0,90])
    union() {
    cube([screen_length,screen_height,10], center = true);
        translate([0,0,4.75])
    cube([screen_length+9,screen_height+1,10], center = true);
    }
}

//keycap_hollow();
