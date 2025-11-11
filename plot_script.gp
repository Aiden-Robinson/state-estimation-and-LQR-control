set terminal png size 800,600
set output 'lqr_control.png'
set title 'LQR Controller - State Trajectory'
set xlabel 'Time (seconds)'
set ylabel 'State Value'
set grid
plot 'OUT' using 1:2 with lines title 'Position', \
     'OUT' using 1:3 with lines title 'Velocity'
