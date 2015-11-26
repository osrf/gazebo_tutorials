% test program that changes color of a sphere when it comes into contact
% with other objects (e.g. hand)


hxs_set_model_collide_mode('sphere_visual_model', 1);
pose = hxs_model_transform('sphere_visual_model');

while (true)
  % note that the color vector is defined by the RGBA 4-tuple:
  %
  % https://s3.amazonaws.com/osrf-distributions/haptix/api/0.7.1/struct__hxsColor.html
  %
  % The elements are ordered such that:
  %   color(1) = float value for the red component
  %   color(2) = float value for the green component
  %   color(3) = float value for the blue component
  %   color(4) = float value for the alpha component
  %
  color = [0;1;0;1];
  if length(hxs_contacts('sphere_visual_model')) > 0,
    color = [1;0;0;1];
  end,
  hxs_set_model_color('sphere_visual_model', color);

  % move sphere around
  t = hx_read_sensors().time_stamp;
  p = pose;
  p.pos(1) = pose.pos(1) - 0.2*cos(1*t);
  hxs_set_model_transform('sphere_visual_model', p);

  sleep(0.001);
endwhile
