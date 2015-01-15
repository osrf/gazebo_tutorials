rqt_plot&
sleep 1
topicnames=(command_age, command_age_mean, delay_in_step, delay_in_window, delay_window_remain)
for topic in ${topicnames[@]}
do
  rqt --command-start-plugin rqt_plot --args /atlas/controller_statistics/"${topic}" &
done
