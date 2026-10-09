## tuning

| error signal | type | settings tried | complete all runs | also meet the specs | kp | ki | kd | tau [s] | RMS error [m] | effort [deg/s] | overshoot [%] | settling [s] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cross-track error | P | 8 | 0 | 0 |  |  |  |  | n/a | n/a | n/a | n/a |
| cross-track error | PD | 168 | 154 | 11 | 3.00 | 0.00 | 4.00 | 0.10 | 0.058 | 152 | 0.0 | 3.95 |
| cross-track error | PI | 48 | 0 | 0 |  |  |  |  | n/a | n/a | n/a | n/a |
| cross-track error | PID | 1008 | 856 | 10 | 3.00 | 0.25 | 3.00 | 0.10 | 0.052 | 150 | 4.1 | 2.65 |
| look-ahead heading error | P | 7 | 7 | 2 | 1.00 | 0.00 | 0.00 | 0.00 | 0.045 | 107 | 6.0 | 3.10 |
| look-ahead heading error | PD | 105 | 105 | 11 | 1.00 | 0.00 | 0.05 | 0.50 | 0.045 | 116 | 4.5 | 2.05 |
| look-ahead heading error | PI | 35 | 35 | 6 | 1.50 | 0.10 | 0.00 | 0.00 | 0.063 | 160 | 0.0 | 2.20 |
| look-ahead heading error | PID | 525 | 525 | 37 | 1.50 | 0.10 | 0.05 | 0.30 | 0.064 | 176 | 0.0 | 2.25 |

## step

| error signal | controller | rise time [s] | overshoot [%] | settling time [s] | largest error [m] |
|---|---|---|---|---|---|
| cross-track error | P (kp=3) | 0.85 | 203.4 | n/a | 1.12 |
| cross-track error | PD (kp=3, kd=4) | 2.70 | 0.0 | 3.95 | 0.50 |
| cross-track error | PI (kp=3, ki=0.25) | 0.85 | 204.8 | n/a | 1.13 |
| cross-track error | PID (kp=3, ki=0.25, kd=3) | 1.65 | 4.1 | 2.65 | 0.50 |
| look-ahead heading error | P (kp=1) | 1.30 | 6.0 | 3.10 | 0.50 |
| look-ahead heading error | PD (kp=1, kd=0.05) | 1.35 | 4.5 | 2.05 | 0.50 |
| look-ahead heading error | PI (kp=1.5, ki=0.1) | 1.35 | 0.0 | 2.20 | 0.50 |
| look-ahead heading error | PID (kp=1.5, ki=0.1, kd=0.05) | 1.45 | 0.0 | 2.25 | 0.50 |

## disturbance

| error signal | type | peak error [mm] | final error [mm] | final ripple [mm] |
|---|---|---|---|---|
| cross-track error | P | 860.8 | -370.9 | 1214.1 |
| cross-track error | PD | 17.5 | 17.5 | 0.0 |
| cross-track error | PI | 973.6 | -642.2 | 990.0 |
| cross-track error | PID | 15.6 | 2.5 | 0.7 |
| look-ahead heading error | P | 28.6 | 26.2 | 0.0 |
| look-ahead heading error | PD | 27.9 | 26.2 | 0.0 |
| look-ahead heading error | PI | 16.3 | 3.8 | 0.8 |
| look-ahead heading error | PID | 16.0 | 3.8 | 0.8 |

## circle

| error signal | type | completed | steady error [mm] | ripple [mm] |
|---|---|---|---|---|
| cross-track error | P | no | 583.1 | 1044.5 |
| cross-track error | PD | yes | -50.3 | 2.0 |
| cross-track error | PI | no | 585.9 | 1039.1 |
| cross-track error | PID | yes | -5.7 | 6.1 |
| look-ahead heading error | P | yes | -14.7 | 4.8 |
| look-ahead heading error | PD | yes | -14.7 | 5.2 |
| look-ahead heading error | PI | yes | 69.1 | 24.2 |
| look-ahead heading error | PID | yes | 69.1 | 25.5 |

## laps

| track | error signal | type | laps finished | RMS error [mm] | sd | max error [mm] | effort [deg/s] | saturated [%] |
|---|---|---|---|---|---|---|---|---|
| cone track | cross-track error | P | 0/20 | n/a | n/a | n/a | n/a | n/a |
| cone track | cross-track error | PD | 20/20 | 28 | 1 | 76 | 148 | 0.6 |
| cone track | cross-track error | PI | 0/20 | n/a | n/a | n/a | n/a | n/a |
| cone track | cross-track error | PID | 20/20 | 33 | 1 | 89 | 147 | 0.4 |
| cone track | look-ahead heading error | P | 20/20 | 19 | 1 | 48 | 105 | 0.1 |
| cone track | look-ahead heading error | PD | 20/20 | 19 | 1 | 48 | 116 | 0.1 |
| cone track | look-ahead heading error | PI | 20/20 | 23 | 1 | 57 | 158 | 0.3 |
| cone track | look-ahead heading error | PID | 20/20 | 23 | 1 | 57 | 174 | 0.4 |
| race track | cross-track error | P | 0/20 | n/a | n/a | n/a | n/a | n/a |
| race track | cross-track error | PD | 20/20 | 19 | 1 | 38 | 147 | 0.0 |
| race track | cross-track error | PI | 0/20 | n/a | n/a | n/a | n/a | n/a |
| race track | cross-track error | PID | 20/20 | 20 | 1 | 38 | 146 | 0.0 |
| race track | look-ahead heading error | P | 20/20 | 15 | 1 | 32 | 105 | 0.1 |
| race track | look-ahead heading error | PD | 20/20 | 15 | 1 | 32 | 114 | 0.1 |
| race track | look-ahead heading error | PI | 20/20 | 28 | 1 | 53 | 158 | 0.2 |
| race track | look-ahead heading error | PID | 20/20 | 28 | 1 | 53 | 174 | 0.3 |

## gain_sweeps

| gain | value | rise time [s] | overshoot [%] | settling time [s] | time to remove trim error [s] | laps finished | lap RMS error [mm] | lap effort [deg/s] |
|---|---|---|---|---|---|---|---|---|
| kp | 0.375 | 3.50 | 59.0 | n/a | n/a | 5/5 | 139 | 28 |
| kp | 0.750 | 3.05 | 36.4 | n/a | 19.8 | 5/5 | 92 | 42 |
| kp | 1.500 | 2.50 | 14.9 | 14.15 | 12.4 | 5/5 | 57 | 77 |
| kp | 3.000 | 1.65 | 4.1 | 2.65 | 15.8 | 5/5 | 33 | 149 |
| kp | 6.000 | 0.90 | 40.2 | 4.50 | 14.0 | 5/5 | 24 | 281 |
| kp | 12.000 | 0.85 | 91.1 | n/a | n/a | 5/5 | 164 | 218 |
| kp | 24.000 | 0.85 | 189.8 | n/a | n/a | 0/5 | n/a | n/a |
| ki | 0.031 | 1.95 | 0.6 | 2.85 | n/a | 5/5 | 33 | 149 |
| ki | 0.063 | 1.90 | 1.2 | 2.80 | n/a | 5/5 | 33 | 149 |
| ki | 0.125 | 1.85 | 2.3 | 2.75 | n/a | 5/5 | 33 | 149 |
| ki | 0.250 | 1.65 | 4.1 | 2.65 | 15.8 | 5/5 | 33 | 149 |
| ki | 0.500 | 1.30 | 7.3 | 7.55 | 8.3 | 5/5 | 33 | 149 |
| ki | 1.000 | 1.15 | 12.0 | 6.85 | 4.8 | 5/5 | 31 | 150 |
| ki | 2.000 | 1.00 | 19.7 | 5.30 | 3.0 | 5/5 | 28 | 151 |
| kd | 0.375 | 0.85 | 142.7 | n/a | n/a | 1/5 | 338 | 112 |
| kd | 0.750 | 0.85 | 83.1 | n/a | n/a | 5/5 | 51 | 147 |
| kd | 1.500 | 0.90 | 40.7 | 5.50 | 15.3 | 5/5 | 41 | 147 |
| kd | 3.000 | 1.65 | 4.1 | 2.65 | 15.8 | 5/5 | 33 | 149 |
| kd | 6.000 | 3.30 | 8.8 | n/a | 16.9 | 5/5 | 22 | 157 |
| kd | 12.000 | 5.80 | 16.6 | n/a | n/a | 5/5 | 56 | 199 |
| kd | 24.000 | 10.10 | 18.0 | n/a | n/a | 5/5 | 99 | 247 |

## d_term

| D term | laps finished | RMS error [mm] | sd | max error [mm] | effort [deg/s] | saturated [%] |
|---|---|---|---|---|---|---|
| difference, tau = 0 s | 10/10 | 310 | 17 | 751 | 911 | 88.7 |
| difference, tau = 0.05 s | 10/10 | 136 | 10 | 352 | 798 | 74.4 |
| difference, tau = 0.1 s | 10/10 | 94 | 6 | 248 | 722 | 62.1 |
| difference, tau = 0.2 s | 10/10 | 70 | 4 | 187 | 601 | 46.8 |
| difference, tau = 0.3 s | 10/10 | 69 | 3 | 198 | 493 | 41.4 |
| difference, tau = 0.5 s | 10/10 | 110 | 12 | 302 | 307 | 55.5 |
| difference, tau = 1 s | 10/10 | 222 | 8 | 530 | 163 | 74.5 |
| measured rate, tau = 0 s | 10/10 | 28 | 1 | 74 | 171 | 0.7 |
| measured rate, tau = 0.05 s | 10/10 | 28 | 1 | 74 | 151 | 0.6 |
| measured rate, tau = 0.1 s | 10/10 | 28 | 1 | 75 | 148 | 0.6 |
| measured rate, tau = 0.2 s | 10/10 | 28 | 1 | 83 | 147 | 1.1 |
| measured rate, tau = 0.3 s | 10/10 | 33 | 2 | 105 | 147 | 4.5 |
| measured rate, tau = 0.5 s | 10/10 | 73 | 4 | 210 | 129 | 39.5 |
| measured rate, tau = 1 s | 10/10 | 188 | 8 | 457 | 101 | 69.1 |

## anti_windup

| anti-windup | ki | overshoot [%] | settling time [s] | largest integral term [deg] | saturated [%] |
|---|---|---|---|---|---|
| on | 0.25 | 7.5 | 3.3 | 6 | 7.8 |
| off | 0.25 | 23.0 | 12.3 | 21 | 9.2 |

## delay

| delay [ms] | servo lag [s] | overshoot [%] | settling time [s] |
|---|---|---|---|
| 0 | 0.08 | 0.0 | 4.0 |
| 0 | 0.20 | 0.0 | 4.2 |
| 50 | 0.08 | 0.0 | 3.9 |
| 50 | 0.20 | 2.6 | 4.4 |
| 100 | 0.08 | 0.9 | 3.9 |
| 100 | 0.20 | 10.4 | n/a |
| 150 | 0.08 | 8.9 | n/a |
| 150 | 0.20 | 16.3 | n/a |
| 200 | 0.08 | 13.3 | n/a |
| 200 | 0.20 | 23.0 | n/a |
| 300 | 0.08 | 24.6 | n/a |
| 300 | 0.20 | 39.8 | n/a |

## lookahead

| look-ahead [m] | rise time [s] | overshoot [%] | settling time [s] |
|---|---|---|---|
| 0.2 | 0.70 | 33.9 | 4.45 |
| 0.3 | 0.80 | 20.4 | 3.60 |
| 0.5 | 1.10 | 7.6 | 3.00 |
| 0.8 | 1.85 | 0.3 | 2.80 |
| 1.2 | 3.25 | 0.0 | 4.80 |

## waypoint

| controller | laps finished in 300 s | lap time [s] | mean heading error [rad] | RMS error [mm] | saturated [%] |
|---|---|---|---|---|---|
| P (kp=2) | 10/10 | 89 | 0.29 | 49 | 42 |
| PD (kd=0.5, raw D) | 1/10 | 189 | 1.42 | 308 | 83 |
| PD (kd=0.5, tau=0.3) | 9/10 | 109 | 0.55 | 103 | 56 |
| PI (ki=0.2) | 10/10 | 91 | 0.32 | 69 | 42 |

## gazebo

| controller | outcome | laps finished | RMS error [mm] | sd | max error [mm] | effort [deg/s] | saturated [%] |
|---|---|---|---|---|---|---|---|
| P | stalled | 0/3 | n/a | n/a | n/a | n/a | n/a |
| PD | finished | 3/3 | 52 | 1 | 120 | 146 | 3.6 |
| PI | stalled | 0/3 | n/a | n/a | n/a | n/a | n/a |
| PID | finished | 3/3 | 59 | 1 | 149 | 144 | 3.6 |
| P look-ahead | finished | 3/3 | 51 | 1 | 124 | 103 | 0.8 |

## notes

kp-kd map: 69 % of the cells never settle.
look-ahead equivalence: cross-track PD gains kp = 2.000, kd = 1.667; largest difference between the two responses 12.9 mm.
