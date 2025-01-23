# This message contains an AckermannDriveStamped control message,
# an ID identifying the sender and two boolean values, indicating
# if the command is intended to be used.

# Sender IDs
int8 MANUAL  = -1
int8 REVERSE =  0
int8 PP      =  1
int8 FTG     =  2

std_msgs/Header header

ackermann_msgs/AckermannDriveStamped drive_parameters
std_msgs/Int8 id
std_msgs/Bool active