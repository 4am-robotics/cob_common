# This message communicates the state of the power system.
Header header
float64 voltage                     # [V]
float64 current                     # [A]
float64 power_consumption           # [W] can only be calculated if not charging
float64 remaining_capacity          # [Ah]
float64 relative_remaining_capacity # [0..100] percent of maximum capacity (parameter max_capacity)
bool charging                       # flag if robot is connected to external power or not
float64 time_remaining              # [h] estimated time to empty or fully charged
float64 temperature                 # [Celsius] temperature of the battery
