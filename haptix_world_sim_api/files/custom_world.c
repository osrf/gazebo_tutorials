/*

Test program that changes color of a sphere model when it comes into contact with other objects (e.g. hand).

Green sphere model moves along table, turns red upon making contact.

*/

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <haptix/comm/haptix_sim.h>
#include <haptix/comm/haptix.h>

#define RADII 6.28318530718

int main(int argc, char **argv) {

int t2, t1 = 0;
float reset = 0.0, time = 0.0;
hxSensor t;
hxsContactPoints contact_pts;
hxsTransform pose, p;
hxsColor color;
hxsCollideMode collide_mode, one = hxsDETECTIONONLY;

if (hx_connect(NULL,0) != hxOK) {
	printf("hx_connect(): Request error.\n");
	return -1;
}

hxs_set_model_collide_mode("sphere_visual_model", &one);
hxs_model_transform("sphere_visual_model", &pose);
p = pose;

while(1) {
	
	// changes sphere color upon contact
	color.alpha = 1.0; color.b = 0.0; color.g = 1.0; color.r = 0.0;
	hxs_contacts("sphere_visual_model", &contact_pts);
	if (contact_pts.contact_count > 0) {
		color.alpha = 1.0; color.b = 0.0; color.g = 0.0; color.r = 1.0; }
	hxs_set_model_color("sphere_visual_model", &color);

	// take simulation time (nsec)
	hx_read_sensors(&t);
	t2 = t.time_stamp.nsec;

	// adjust time so that time_stamp reset does not also reset sphere position
	if (!t2 && t1) {			// if time_stamp reset
		reset = time; }			// then update 'reset tracker'

	// adjust position of sphere
	time = fmod(0.01*t2, RADII) + reset;
	p.pos.x = pose.pos.x - 0.2*cos(time);
	hxs_set_model_transform("sphere_visual_model", &p);
	t1 = t2;

	nanosleep(1000000); }

return 0;
}
