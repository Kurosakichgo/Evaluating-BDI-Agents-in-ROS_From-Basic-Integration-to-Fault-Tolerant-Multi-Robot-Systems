goal(goal1, 2.5, 1.0).
goal(goal2, -6.0, -0.5).
goal(goal3, 5.0, 1.0).
vl(0).
bids([]).

!start_auction.

@plan[atomic]
+bid(A, GoalId, GX, GY, Distance) : bids(Bids) & .length(Bids,Len) <-
.print("coordinator received goal: (", Distance, ", ", GY, ")");
.concat(Bids,[bid(A, GoalId, GX, GY, Distance)],NewBids); 
.abolish(bids(Bids));
+bids(NewBids);
.print(NewBids);
if (Len == 8) {
    .print("Test");
    select_best_bid(NewBids);
}.


+best_bid(A, GoalId, GX, GY, Distance) <-
 .print("Best bid: ", bid(A, GoalId, GX, GY, Distance));
 .send(A, tell, assign_goal(GoalId, GX, GY)).


+!start_auction <-
    .print("Starting auction...");
    !send_goals.

+!send_goals <-
    .send(agent1, tell, goal(goal1, 2.5, 1.0));
    .send(agent2, tell, goal(goal1, 2.5, 1.0));
    .send(agent3, tell, goal(goal1, 2.5, 1.0));
    .wait(1000);
    .send(agent1, tell, goal(goal2, -6.0, -0.5));
    .send(agent3, tell, goal(goal2, -6.0, -0.5));
    .send(agent2, tell, goal(goal2, -6.0, -0.5));
    .wait(1000);
    .send(agent2, tell, goal(goal3, 5.0, 1.0));
    .send(agent1, tell, goal(goal3, 5.0, 1.0));
    .send(agent3, tell, goal(goal3, 5.0, 1.0)).


