% This example exercises every simulation API call.

info = hxs_sim_info();

% List the models in the world
fprintf('Models:\n');
for model_idx = 1:size(info.models)
  model = info.models(model_idx).model;
  fprintf('  %s\n', model.name);
  fprintf('    Links:\n');
  for link_idx = 1:size(model.links)
    link = model.links(link_idx).link;
    fprintf('      %s\n', link.name);
  end
  fprintf('    Joints:\n');
  for joint_idx = 1:size(model.joints)
    joint = model.joints(joint_idx).joint;
    fprintf('      %s\n', joint.name);
  end
end

% Get the user camera pose
tx = hxs_camera_transform();
% Move and rotate the user camera pose
new_tx = tx;
new_tx.pos(3) = new_tx.pos(3) + 1;
new_tx.orient(1)= new_tx.orient(1) + pi/4;
hxs_set_camera_transform(new_tx);
% Restore the original camera pose
pause(1);
hxs_set_camera_transform(tx);
pause(1);

% Change the table color.
hxs_set_model_color('table', [1;0;0;1])
pause(1);
% Row vectors work, too
hxs_set_model_color('table', [0,1,0,1])
pause(1);
hxs_set_model_color('table', [0;0;1;1])
% Get the color
color = hxs_model_color('table');
disp('Table color:');
disp(color)

% Apply force to the small wooden cube, moving it sideways.
% Show linear velocity before, during, and afterward
disp('Sliding cube:')
vel = hxs_linear_velocity('wood_cube_5cm');
disp(vel);
hxs_apply_force('wood_cube_5cm', 'link', [-1.0; 0; 0], 0.2);
% Let it get moving
pause(0.1);
vel = hxs_linear_velocity('wood_cube_5cm');
disp(vel);
% Let it settle
pause(1.5);
vel = hxs_linear_velocity('wood_cube_5cm');
disp(vel);

% Apply torque to the small wooden cube, rotating it in place.
% Show angular velocity before, during, and afterward
disp('Spinning cube:')
vel = hxs_angular_velocity('wood_cube_5cm');
disp(vel);
hxs_apply_torque('wood_cube_5cm', 'link', [0; 0; 0.1], 0.1)
% Let it get moving
pause(0.1);
vel = hxs_angular_velocity('wood_cube_5cm');
disp(vel);
% Let it settle
pause(1.5);
vel = hxs_angular_velocity('wood_cube_5cm');
disp(vel);

% Apply force and torque at the same time.
wrench = struct('force', [0; 0; 1], 'torque', [0; 0; 0.1]);
hxs_apply_wrench('wood_cube_5cm', 'link', wrench, 0.1);
pause(1.5);

% Move by setting linear velocity
hxs_set_linear_velocity('wood_cube_5cm', [-0.5; 0; 0]);
pause(1.0);
% Row vectors work, too.
hxs_set_linear_velocity('wood_cube_5cm', [0, 0, 0]);

% Move by setting angular velocity
hxs_set_angular_velocity('wood_cube_5cm', [0; 0; 100]);
pause(1.0);
% Row vectors work, too.
hxs_set_angular_velocity('wood_cube_5cm', [0, 0, 0]);

% Check gravity mode on wooden cube
gravity_mode = hxs_model_gravity_mode('wood_cube_5cm');
disp(gravity_mode);
% Turn off gravity for cube, then nudge it upward
hxs_set_model_gravity_mode('wood_cube_5cm', 0);
% Row vectors work, too.
hxs_apply_force('wood_cube_5cm', 'link', [0, 0, 0.1], 0.1);
% Let if fly
pause(1);
% Bring it back down
hxs_set_model_gravity_mode('wood_cube_5cm', gravity_mode);

% Get the pose of the cube
tx = hxs_model_transform('wood_cube_5cm');
disp('Cube position:');
disp(tx.pos);
disp('Cube orientation:');
disp(tx.orient);
% Modify and set the pose
tx.pos(2) = tx.pos(2) + 0.25;
tx.orient(2)= tx.orient(2) + pi/4;
hxs_set_model_transform('wood_cube_5cm', tx);

% Check collide mode on the cube
collide_mode = hxs_model_collide_mode('wood_cube_5cm');
disp(collide_mode);
% Let it drop through the world
hxs_set_model_collide_mode('wood_cube_5cm', 0);
% Hack: apply a small force to disturb the cube to make it actually fall.
hxs_apply_force('wood_cube_5cm', 'link', [0; 0; 0.1], 0.1);
pause(1);
% Turn collisions back on (won't bring the cube back, of course)
hxs_set_model_collide_mode('wood_cube_5cm', collide_mode);

% Define a new model.  Here, we're taking the cricket_ball model from:
%  https://bitbucket.org/osrf/gazebo_models/src/default/cricket_ball/model.sdf
% and tweaking it slightly (just changing the color from Red to Green).
sdf = '<sdf version="1.5"> <model name="cricket_ball"> <link name="link"> <pose>0 0 0.0375 0 0 0</pose> <inertial> <mass>0.1467</mass> <inertia> <ixx>8.251875e-05</ixx> <ixy>0</ixy> <ixz>0</ixz> <iyy>8.251875e-05</iyy> <iyz>0</iyz> <izz>8.251875e-05</izz> </inertia> </inertial> <collision name="collision"> <geometry> <sphere> <radius>0.0375</radius> </sphere> </geometry> </collision> <visual name="visual"> <geometry> <sphere> <radius>0.0375</radius> </sphere> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Green</name> </script> </material> </visual> </link> </model> </sdf>';
% Add the new model to the world, at the world origin, 5m off the ground, with
% gravity enabled.  Then it will drop onto the table.
hxs_add_model(sdf, 'green_cricket_ball', [0; 0; 5], [0; 0; 0], 1);
pause(2);
% Roll the ball to the right
hxs_apply_torque('green_cricket_ball', 'link', [0; 0.05; 0], 0.1)
pause(2);
% Remove the model
hxs_remove_model('green_cricket_ball');

% Set the state of a wrist joint.  Note that, because there's a controller
% acting on the wrist, this change will only be transient; the controller will
% restore the wrist back to the current target position.
hxs_set_model_joint_state('mpl_haptix_right_forearm', 'wristy', 0.5, 0.0);
pause(1);

% Set the position of the arm. Note that if the motion tracking device is
% active and unpaused, this change will be transient.
hxs_set_model_transform('mpl_haptix_right_forearm', 1.0, 0, 1.5, 0, 0, 0)

% Move the camera
hxs_set_camera_transform(new_tx);
pause(1);
% Reset the world, which will move the camera back
hxs_reset(1);
pause(1);
