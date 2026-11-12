# Week 6 — Localization / encoder odometry, student ID 39

Open `Husanboy_trajectory.qrs` in TRIK Studio, or run `Husanboy_trajectory.py` in Python mode. These are alternative implementations of the same motion plan, not programs to run together. One QRS serves both simulator and robot, as requested; archive a copy under the course's simulator/real names if separate uploads are required.

The controller is **not PID**, including no P-only steering. It turns in place and drives straight at fixed motor power; signed encoder thresholds stop each wheel separately. Position and heading are observed for logging only. All 14 segments are included, including the bottom of 9. Turns and the connector between digits lift the simulator marker. The Python version also waits 100 ms after each motion to log settling; QRS uses its Stop Motors blocks between motions. Both abort motion after a 30-second action timeout rather than waiting forever.

## Run and submit

1. Use M3/E3 for the left wheel and M4/E4 for the right wheel. Positive power must give positive forward encoder travel. Measure radius and track width; defaults are 0.028 m and 0.154 m. Update the Python constants or the QRS initialization block consistently. Start at the top-left of 3, facing +x, with clear floor around the approximately 1.05 m by 0.60 m drawing. The simulator world is reset to origin, heading zero, without obstacles.
2. Run the same selected program in the simulator. Save its `Husanboy_odometry_data.txt` as `Husanboy_odometry_data_sim.txt` before another run overwrites it.
3. Run on the robot with the same settings. Save the output as `Husanboy_odometry_data_real.txt`. Logs have no header: `x,y,time`, in metres/metres/milliseconds. The task's unprefixed filenames correspond to these Husanboy-prefixed names.
4. Open `Husanboy_analysis.ipynb` locally or in Colab and provide both fresh logs. Different sample counts are supported: interpolation uses only the shared time interval. Timing differences can affect this comparison; the notebook also reports final endpoint difference and separate durations.
5. Measure the final axle midpoint with a tape relative to the starting point (+x initially forward, +y left). Enter the coordinates in the notebook, then complete the five-sentence discussion using actual observations.

Fresh logs and physical measurements remain pending; no experimental results are invented. Equal fixed powers can drift on mismatched real motors. Wheel slip, wrong wheel geometry and unequal wheel travel accumulate in the encoder-only pose estimate, as this lab is intended to study.

The previous waypoint controller and its existing log were moved intact to `../week7/` (with week labels corrected). They are not evidence for this new program.
