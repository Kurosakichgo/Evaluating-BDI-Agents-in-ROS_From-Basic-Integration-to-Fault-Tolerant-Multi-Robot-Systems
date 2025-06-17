
+robot_at(home).
!start.

+!move_forward : true <-
    .print("Moving forward");
    move_forward.

+!move_left : true <-
    .print("Moving left");
    move_left.

+!move_right : true <-
   .print("Moving right");
   move_right;
.

+!stop : true <-
   .print("Stop");
   stop;
.

+!wait : wait <-
   .print("wait");
   .wait(500).

+!start <-
    .print("Start");
    !move_forward;
    .wait(8700);
    !stop;
    !move_left;
    .wait(3200);
    !stop;
    !move_forward;
    .wait(2700);
    !stop;
    !move_left;
    .wait(3200);
    !move_forward;
    .wait(7900);
    !stop;
    !move_right;
    .wait(3280);
    !stop;
    !move_forward;
    .wait(2700);
    !stop;
    !move_right;
    .wait(3200);
    !stop;
    !move_forward;
    .wait(8700);
    !stop.

    


    
