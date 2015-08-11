% test program that changes color of a sphere when it comes into contact
% with other objects (e.g. hand)


hxs_set_model_collide_mode('sphere_visual_model', 1)

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
  sleep(0.1);
endwhile
