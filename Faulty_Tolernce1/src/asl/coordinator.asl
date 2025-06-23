goal(goal1, 2.5, 1.0).
goal(goal2, -6.0, -0.5).
goal(goal3, 5.0, 1.0).
goal(goal4, 5.0, 4.0).
goal(goal5, -2.5, 2.0).
goal(goal6, 6.0, 3.0).
vl(0).
bids([]).
bidf([]).
agents([agent1,agent2,agent3,agent4,agent5,agent6]).

!start_auction.

@plan[atomic]
+bid(A, GX, GY, Distance) : bids(Bids) & .length(Bids,Len) <-
.print("coordinator received goal: (", Distance, ", ", GY, ")");
.concat(Bids,[bid(A, GX, GY, Distance)],NewBids); 
.abolish(bids(Bids));
+bids(NewBids);
.print(NewBids);
if (Len == 35) {
    .print("Test");
    select_best_bid(NewBids);
}.



+best_bid(A, GX, GY, Distance) <-
 .print("Best bid: ", bid(A, GX, GY, Distance));
 .send(A, tell, assign_goal(GX, GY)).


+!start_auction <-
    .print("Starting auction...");
    !send_goals.

+!send_goals <-
     .send(agent1, tell, goal(goal1, 2.5, 1.0));
    .send(agent2, tell, goal(goal1, 2.5, 1.0));
    .send(agent3, tell, goal(goal1, 2.5, 1.0));
    .send(agent4, tell, goal(goal1, 2.5, 1.0));
    .send(agent5, tell, goal(goal1, 2.5, 1.0));
    .send(agent6, tell, goal(goal1, 2.5, 1.0));
    .wait(1000);
    .send(agent1, tell, goal(goal2, -6.0, -0.5));
    .send(agent3, tell, goal(goal2, -6.0, -0.5));
    .send(agent2, tell, goal(goal2, -6.0, -0.5));
    .send(agent4, tell, goal(goal2, -6.0, -0.5));
    .send(agent5, tell, goal(goal2, -6.0, -0.5));
    .send(agent6, tell, goal(goal2, -6.0, -0.5));
    .wait(1000);
    .send(agent2, tell, goal(goal3, 5.0, 1.0));
    .send(agent1, tell, goal(goal3, 5.0, 1.0));
    .send(agent3, tell, goal(goal3, 5.0, 1.0));
    .send(agent4, tell, goal(goal3, 5.0, 1.0));
    .send(agent5, tell, goal(goal3, 5.0, 1.0));
    .send(agent6, tell, goal(goal3, 5.0, 1.0));
    .wait(1000);
    .send(agent1, tell, goal(goal4, 5.0, 4.0));
    .send(agent2, tell, goal(goal4, 5.0, 4.0));
    .send(agent3, tell, goal(goal4, 5.0, 4.0));
    .send(agent4, tell, goal(goal4, 5.0, 4.0));
    .send(agent5, tell, goal(goal4, 5.0, 4.0));
    .send(agent6, tell, goal(goal4, 5.0, 4.0));
    .wait(1000);
    .send(agent1, tell, goal(goal5, -2.5, 2.0));
    .send(agent2, tell, goal(goal5, -2.5, 2.0));
    .send(agent3, tell, goal(goal5, -2.5, 2.0));
    .send(agent4, tell, goal(goal5, -2.5, 2.0));
    .send(agent5, tell, goal(goal5, -2.5, 2.0));
    .send(agent6, tell, goal(goal5, -2.5, 2.0));
    .wait(1000);
    .send(agent2, tell, goal(goal6,  6.0, 3.0));
    .send(agent1, tell, goal(goal6,  6.0, 3.0));
    .send(agent3, tell, goal(goal6,  6.0, 3.0));
    .send(agent4, tell, goal(goal6,  6.0, 3.0));
    .send(agent5, tell, goal(goal6,  6.0, 3.0));
    .send(agent6, tell, goal(goal6,  6.0, 3.0)).


    
@plan2[atomic]
+bid_f(A, GX, GY, Distance) : bidf(Bids) & .length(Bids,Len) <-
.print("coordinator received goal: (", Distance, ", ", GY, ")");
.concat(Bids,[bid_f(A, GX, GY, Distance)],NewBids); 
.abolish(bidf(Bids));
+bidf(NewBids);
.print(NewBids);
if (Len == 4) {
    .print("Test");
    select_best_bid(NewBids);
}.


+best_bid_f(A, GX, GY, Distance) <-
 .print("Best bid: ", bid(A, GX, GY, Distance));
 .send(A, tell, rescue(GX, GY)).
