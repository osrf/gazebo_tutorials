/*

Converted from MATLAB to C.
This example exercises every simulation API call.

*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <haptix/comm/haptix_sim.h>
#include <haptix/comm/haptix.h>

#define PI 3.1415926

int main(int argc, char **argv) {

// 'malloc' is necessary for hxSimInfo variables
hxsSimInfo * info = (hxsSimInfo *) malloc(sizeof(hxsSimInfo));

hxsModel model;
hxsLink link;
hxsJoint joint;

hxsTransform tx, new_tx;
hxsColor color;
hxsContactPoints contact_pts;
hxsVector3 point, vel, force;
hxsWrench wrench;
int gravity_mode, model_idx, link_idx, joint_idx, contact_idx;

// Connect to arat.world
if (hx_connect(NULL,0) != hxOK) {
	printf("hx_connect(): Request error.\n");
	return -1; }

// List the models in the world
hxs_sim_info(info);
printf("Models:\n");
for (model_idx = 0; model_idx < info->model_count; model_idx++) {
	model = info->models[model_idx];
	printf("  %s\n", model.name);
	printf("    Links:\n");
	for (link_idx = 0; link_idx < model.link_count; link_idx++) {
		link = model.links[link_idx];
		printf("      %s\n", link.name); }
	printf("    Joints:\n");
	for (joint_idx = 0; joint_idx < model.joint_count; joint_idx++) {
		joint = model.joints[joint_idx];
		printf("      %s\n", joint.name); }
}

hxs_camera_transform(&tx);			// Get the user camera pose
new_tx = tx;				
new_tx.pos.z = new_tx.pos.z + 1;		// Move and rotate the user camera pose
new_tx.orient.w = new_tx.orient.w + 0.25*PI;
hxs_set_camera_transform(&new_tx);
usleep(1000000);
hxs_set_camera_transform(&tx);			// Restore the original camera pose
usleep(1000000);

color.alpha = 1.0; color.b = 0.0; color.g = 0.0; color.r = 1.0;
hxs_set_model_color("table", &color);			// Change the table color
usleep(1000000);
color.alpha = 1.0; color.b = 0.0; color.g = 1.0; color.r = 0.0;
hxs_set_model_color("table", &color);
usleep(1000000);
color.alpha = 1.0; color.b = 1.0; color.g = 0.0; color.r = 0.0;
hxs_set_model_color("table", &color);
hxs_model_color("table", &color);			// Get the color
printf("Table color:\n   %f\n   %f\n   %f\n   %f\n", color.alpha, color.b, color.g, color.r);

// Get contact information for the cube after collision
force.x = 0; force.y = 0; force.z = -1;			// Direct force towards table
hxs_apply_force("wood_cube_5cm", "link", &force, 0.1);	// Applying a force here induces a collision
usleep(50000);
hxs_contacts("wood_cube_5cm", &contact_pts);
printf("Contact points:\n");
for (contact_idx = 0; contact_idx < contact_pts.contact_count; contact_idx++) {
	printf("  Contact %d:\n", contact_idx + 1);
	printf("    Link 1: %s\n", contact_pts.contacts[contact_idx].link1);
	printf("    Link 2: %s\n", contact_pts.contacts[contact_idx].link2);
	point = contact_pts.contacts[contact_idx].point;
	printf("    Contact point: %g, %g, %g\n", point.x, point.y, point.z);
	printf("    Contact penetration depth: %g\n", contact_pts.contacts[contact_idx].distance); }
usleep(1000000);

// Apply force to the small wooden cube, moving it sideways.
printf("Sliding cube:\n");				// Show linear velocity before, during, and afterward
hxs_linear_velocity("wood_cube_5cm", &vel);
printf("  %- 11.5g\n  %- 11.5g\n  %- 11.5g\n", vel.x, vel.y, vel.z);
force.x = -1.0; force.y = 0; force.z = 0;
hxs_apply_force("wood_cube_5cm", "link", &force, 0.2);	// Let it move
usleep(100000);						// Let it settle
hxs_linear_velocity("wood_cube_5cm", &vel);
printf("  %- 11.5g\n  %- 11.5g\n  %- 11.5g\n", vel.x, vel.y, vel.z);
usleep(1500000);
hxs_linear_velocity("wood_cube_5cm", &vel);
printf("  %- 11.5g\n  %- 11.5g\n  %- 11.5g\n", vel.x, vel.y, vel.z);

// Apply torque to the small wooden cube, rotating it in place.
printf("Spinning cube:\n");				// Show angular velocity before, during, and afterward
hxs_angular_velocity("wood_cube_5cm", &vel);
printf("  %- 11.5g\n  %- 11.5g\n  %- 11.5g\n", vel.x, vel.y, vel.z);
force.x = 0; force.y = 0; force.z = 0.1;
hxs_apply_torque("wood_cube_5cm", "link", &force, 0.1);	// Let it move
usleep(100000);						// Let it settle
hxs_angular_velocity("wood_cube_5cm", &vel);
printf("  %- 11.5g\n  %- 11.5g\n  %- 11.5g\n", vel.x, vel.y, vel.z);
usleep(1500000);
hxs_angular_velocity("wood_cube_5cm", &vel);
printf("  %- 11.5g\n  %- 11.5g\n  %- 11.5g\n", vel.x, vel.y, vel.z);

// Apply force and torque at the same time.
wrench.force.x = 0; wrench.force.y = 0; wrench.force.z = 1;
wrench.torque.x = 0; wrench.torque.y = 0; wrench.torque.z = 0.1;
hxs_apply_wrench("wood_cube_5cm", "link", &wrench, 0.1);
usleep(1500000);

// Move by setting linear velocity
force.x = -0.5; force.y = 0; force.z = 0;
hxs_set_linear_velocity("wood_cube_5cm",&force);		
usleep(1000000);

// Move by setting angular velocity
force.x = 0; force.y = 0; force.z = 100;
hxs_set_angular_velocity("wood_cube_5cm", &force);		
usleep(1000000);

hxs_model_gravity_mode("wood_cube_5cm", &gravity_mode);		// Check gravity mode on wooden cube
printf("Gravity mode: %d\n", gravity_mode);
hxs_set_model_gravity_mode("wood_cube_5cm", 0);			// Turn off gravity for cube
force.x = 0; force.y = 0; force.z = 0.1;
hxs_apply_force("wood_cube_5cm", "link", &force, 0.1);		// Nudge it upwards
usleep(1000000);						// Let it fly
hxs_set_model_gravity_mode("wood_cube_5cm", gravity_mode);	// Bring it back down

hxs_model_transform("wood_cube_5cm",&tx);	// Get the pose of the cube
printf("Cube position:\n  %- 11.7f\n  %- 11.7f\n  %- 11.7f\n", tx.pos.x, tx.pos.y, tx.pos.z);
printf("Cube orientation:\n  %- 13.4e\n  %- 13.4e\n  %- 13.4e\n  %- 13.4e\n", tx.orient.w, tx.orient.x, tx.orient.y, tx.orient.z);
tx.pos.y = tx.pos.y + 0.25;
tx.orient.x = tx.orient.x + PI/4;
hxs_set_model_transform("wood_cube_5cm", &tx);


hxsCollideMode collide_mode, non = hxsNOCOLLIDE;
hxs_model_collide_mode("wood_cube_5cm", &collide_mode);		// Check collide_mode on the cube
printf("Collide mode: %d\n", collide_mode);
hxs_set_model_collide_mode("wood_cube_5cm", &non);		// Let it drop through the table
force.x = 0; force.y = 0; force.z = 0.1;
hxs_apply_force("wood_cube_5cm", "link", &force, 0.1);		// Hack: apply a small force to disturb the cube to make it actually fall.
usleep(1000000);
hxs_set_model_collide_mode("wood_cube_5cm", &collide_mode);	// Turn collisions back on (won't bring the cube back, of course)

// Critical error occurs when attempting to run hx_controller.c twice in a row: results in 'Ogre::ItemIdentityException'
// Error easily avoided by commenting out addition of green_cricket_ball model
/*
// Define a new model.  Here, we're taking the cricket_ball model from: <https://bitbucket.org/osrf/gazebo_models/src/default/cricket_ball/model.sdf> and tweaking it slightly (just changing the color from Red to Green).
const char sdf[] = "<sdf version=\"1.5\"> <model name=\"cricket_ball\"> <link name=\"link\"> <pose>0 0 0.0375 0 0 0</pose> <inertial> <mass>0.1467</mass> <inertia> <ixx>8.251875e-05</ixx> <ixy>0</ixy> <ixz>0</ixz> <iyy>8.251875e-05</iyy> <iyz>0</iyz> <izz>8.251875e-05</izz> </inertia> </inertial> <collision name=\"collision\"> <geometry> <sphere> <radius>0.0375</radius> </sphere> </geometry> </collision> <visual name=\"visual\"> <geometry> <sphere> <radius>0.0375</radius> </sphere> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Green</name> </script> </material> </visual> </link> </model> </sdf>";

hxs_add_model(sdf, "green_cricket_ball", 0, 0, 5, 0, 0, 0, 1, &model);	// Add the new model to the world, at the world origin, 5m off the ground, with gravity enabled.  Then it will drop onto the table.
usleep(2000000);
force.x = 0; force.y = 0.05; force.z = 0;
hxs_apply_torque("green_cricket_ball", "link", &force, 0.1);
usleep(2000000);
hxs_remove_model("green_cricket_ball");					//  Remove the model
*/

// Set the state of a wrist joint.  Note that, because there's a controller acting on the wrist, this change will only be transient; the controller will restore the wrist back to the current target position.
hxs_set_model_joint_state("mpl_haptix_right_forearm","wristy", 0.5, 0.0);
usleep(1000000);

// Set the position of the arm. Note that if the motion tracking device is active and unpaused, this change will be transient.
tx.pos.x = 1.0; tx.pos.y = 0; tx.pos.z = 1.5;
tx.orient.w = 1; tx.orient.x = 0; tx.orient.y = 0; tx.orient.z = 0;
hxs_set_model_transform("mpl_haptix_right_forearm", &tx);

hxs_set_camera_transform(&new_tx);	// Move the camera
usleep(1000000);
hxs_reset(1);				// Reset the world, which will move the camera back
usleep(1000000);

if (hx_close() != hxOK){
	printf("hx_close(): Request error.\n");
	return -1; }

free(info);
return 0; }
