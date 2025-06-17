my_name(agent1).
my_position(1.5, 0.5).
my_goal(none).
current_goal(none,0,0).
paused_goal(none,0,0).

+goal(GoalId, GX, GY) <-
.print("Agent1 received goal: (", GX, ", ", GY, ")");
?my_position(X, Y);
?my_name(A);
calculate_distance_for_bid( X, Y, GX, GY, A, GoalId).


+distance(GoalId, A, GX, GY, Distance) <-
.send(coordinator, tell, bid(A, GoalId, GX, GY, Distance)).

+assign_goal(GoalId, GX, GY) <-
    .print("Agent1 assigned to goal: (", GX, ",", GY, ")");
    send_goal(GX, GY, "tb1").
 


/* 2. 救援计划：收到 rescue 事件后，挂起原意图，切换到救援 */
+rescue(Agent, GoalId, X, Y) : Agent == id <-
    .print("Agent", id, "rescues goal", GoalId);           
    !navigate_rescue(X, Y).          

+!navigate_rescue(X, Y) <-
    .print("Agent", id, "going to rescue at", X, Y);
    send_goal(X, Y, id);
    .print("Agent", id, "rescue complete").

+resume(Agent, GoalId) : Agent == id <-
    .print("Agent", id, "resume goal", GoalId);
    !navigate(GoalId).                  