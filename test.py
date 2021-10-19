import robosim

field_type = 0
n_robots_blue = 3
n_robots_yellow = 3
time_step_ms = 25

ball_pos = [0, 0, 0, 0]
blue_robots_pos = [[-0.2 * i, 0, 0]
                   for i in range(1, n_robots_blue + 1)]
yellow_robots_pos = [[0.2 * i, 0, 0]
                     for i in range(1, n_robots_yellow + 1)]

robosim.VSS(field_type, n_robots_blue, n_robots_yellow, time_step_ms, ball_pos, blue_robots_pos, yellow_robots_pos)