goal(goal1, 2.5, 1.0).
goal(goal2, -6.0, -0.5).
goal(goal3, 5.0, 1.0).
vl(0).
bids([]).

!start_auction.


+bid(A, GoalId, GX, GY, Distance) <-
.print("coordinator received goal: (", Distance, ", ", GY, ")");
?bids(Bids); 
append(bid(A, GoalId, GX, GY, Distance)); 
-+bids(Bids);
.print(Bids).







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


