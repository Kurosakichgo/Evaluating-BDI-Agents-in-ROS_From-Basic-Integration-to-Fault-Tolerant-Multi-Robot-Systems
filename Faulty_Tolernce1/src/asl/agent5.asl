my_name(agent5).
my_position(4.2, -1.5).
my_goal(none).
current_goal(none,0,0).
paused_goal(none,0,0).

+goal(GoalId, GX, GY) <-
.print("Agent5 received goal: (", GX, ", ", GY, ")");
?my_position(X, Y);
?my_name(A);
calculate_distance_for_bid( X, Y, GX, GY, A).


+distance(A, GX, GY, Distance) <-
.send(coordinator, tell, bid(A, GX, GY, Distance)).

+assign_goal(GX, GY) <-
    .print("Agent5 assigned to goal: (", GX, ",", GY, ")");
    -current_goal(_, _);
    +current_goal(GX, GY);
    send_goal(GX, GY, "tb5").
 
+failure_event(FailedAgent, X, Y)  : FailedAgent \== self  <-
    .print("Agent5: failure of", FailedAgent, "for goal");
    ?current_goal(CX, CY);
    -current_goal(CX, CY);
    +paused_goal(CX, CY);
    ?my_position(GX, GY);
    ?my_name(A);
    calculate_distance_for_bid_f(GX, GY, X, Y, A).
    
+distance_f(A, GX, GY, Distance) <-
    .send(coordinator, tell, bid_f(A, GX, GY, Distance)).

+rescue(GX, GY) <-
    .print("Agent5: executing rescue for goal: (", GX, ",", GY, ")");
    send_goal(GX, GY, "tb1");
    wait(3000);
    ?paused_goal(PX, PY);
    -paused_goal(PX, PY);
    -current_goal(_, _);
    +current_goal(PX, PY);
    send_goal(PX, PY, "tb5");
    -current_goal(X, Y).

 