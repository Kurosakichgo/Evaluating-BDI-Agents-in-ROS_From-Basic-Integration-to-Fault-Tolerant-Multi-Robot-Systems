my_name(agent2).
my_position(-1.5, 2.5).
my_goal(none).

+goal(GoalId, GX, GY) <-
.print("Agent1 received goal: (", GX, ", ", GY, ")");
?my_position(X, Y);
?my_name(A);
calculate_distance_for_bid( X, Y, GX, GY, A, GoalId).


+distance(GoalId, A, GX, GY, Distance) <-
.send(coordinator, tell, bid(A, GoalId, GX, GY, Distance));
send_goal(6.0,2.5,"tb1").

+assign_goal(GoalId, GX, GY) <-
    .print("Agent1 assigned to goal: (", GX, ",", GY, ")").
 