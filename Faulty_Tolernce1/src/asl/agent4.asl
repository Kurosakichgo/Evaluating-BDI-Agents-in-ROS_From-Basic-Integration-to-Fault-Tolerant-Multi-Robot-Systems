
my_name(agent4).
my_position(1.5, -1.5).
my_goal(none).
current_goal(none,0,0).
paused_goal(none,0,0).

+goal(GoalId, GX, GY) <-
.print("Agent4 received goal: (", GX, ", ", GY, ")");
?my_position(X, Y);
?my_name(A);
calculate_distance_for_bid( X, Y, GX, GY, A).


+distance(A, GX, GY, Distance) <-
.send(coordinator, tell, bid(A, GX, GY, Distance)).

+assign_goal(GX, GY) <-
    .print("Agent4 assigned to goal: (", GX, ",", GY, ")");
    -current_goal(_, _);
    +current_goal(GX, GY);
    send_goal(GX, GY, "tb4").
 
+failure_event(FailedAgent, X, Y)  : FailedAgent \== self  <-
    .print("Agent4: failure of", FailedAgent, "for goal");
    ?current_goal(CX, CY);
    -current_goal(CX, CY);
    +paused_goal(CX, CY);
    ?my_position(GX, GY);
    ?my_name(A);
    calculate_distance_for_bid_f(GX, GY, X, Y, A).
    
+distance_f(A, GX, GY, Distance) <-
    .send(coordinator, tell, bid_f(A, GX, GY, Distance)).

+rescue(GX, GY) <-
    .print("Agent4: executing rescue for goal: (", GX, ",", GY, ")");
    send_goal(GX, GY, "tb1");
    wait(3000);
    ?paused_goal(PX, PY);
    -paused_goal(PX, PY);
    -current_goal(_, _);
    +current_goal(PX, PY);
    send_goal(PX, PY, "tb4");
    -current_goal(X, Y).
