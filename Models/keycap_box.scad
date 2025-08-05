// Outer cube dimensions
outer_x = 16;
outer_y = 16;
outer_z = 6;

// Hollowed top section dimensions (plate)
hollow_x = 14;
hollow_y = 14;
hollow_z = 1.5;

// Hollowed out mid section (body)
hollow_x_bod = 14.5;
hollow_y_bod = 14.5;
hollow_z_bod = 4.6 - hollow_z;

// Smooth Circles
$fn = 60;

module keycap() {
    // Build the model
    difference() {
        // Outer solid cube
        cube([outer_x, outer_y, outer_z]);
        keycap_hollow();
    }
}

module keycap_hollow() {
        // Hollowed out top section
    translate([
        (outer_x - hollow_x)/2,
        (outer_y - hollow_y)/2,
        outer_z - hollow_z
    ])
        cube([hollow_x, hollow_y, hollow_z], center=false);
    // Hollowed out bottom section
    translate([
        (outer_x - hollow_x_bod)/2,
        (outer_y - hollow_y_bod)/2,
        outer_z - hollow_z_bod - hollow_z
    ])
        cube([hollow_x_bod, hollow_y_bod, hollow_z_bod], center=false);
    
    // Mounting Holes
        translate([
        (outer_x)/2,
        (outer_y)/2,
        0
    ])
    union() {
       cylinder(3, d=4);
        //translate([4.75,0,0])
        translate([5,0,0])
        cylinder(3, d=2);
        //translate([-4.75,0,0])
        translate([-5,0,0])
        cylinder(3, d=2);
        // pin 1
        translate([-3.5,2.25,0])
        cylinder(3, d=2);
        // pin 2
        translate([2,5,0])
        cylinder(3, d=2);
}; 
translate([
        (outer_x - hollow_x)/2,
        (outer_y - hollow_y)/2,
        outer_z - hollow_z - 5
    ])
        cube([hollow_x, hollow_y, hollow_z+10], center=false);

}

keycap();

//keycap_hollow();
