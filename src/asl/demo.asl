+robot_at(home).
!start.

+!move_forward : true <-
    .print("Moving forward");
    move_forward.

+!stop : true <-
   .print("Stop");
   stop.

+!start <-
    .print("Start");
    !move_forward;
    .wait(8700);
    !stop.