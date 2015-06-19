% test program that changes color of a sphere when it comes into contact
% with other objects (e.g. hand)


hxs_set_model_collide_mode('sphere_visual_model', 1)

while (true)
  color = [0;1;0;1];
  if length(hxs_contacts('sphere_visual_model')) > 0,
    color = [1;0;0;1];
  end,
  hxs_set_model_color('sphere_visual_model', color);
  sleep(0.1);
endwhile
