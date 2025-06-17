goal(goal1, 2.5, 1.0).
goal(goal2, -6.0, -0.5).
goal(goal3, 5.0, 1.0).
vl(0).
bids([]).
agents([agent1,agent2,agent3,agent4,agent5,agent6]).

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


/* 1. 感知到环境中新增的 failure_event */

    
+failure_event(FailedAgent, X, Y) <-
    .print("Coordinator: failure of", FailedAgent, "for goal");
    !rescue_auction(FailedAgent, X, Y).

+!rescue_auction(FailedAgent, X, Y) <-
    .print("Coordinator: starting rescue auction for", "excluding", FailedAgent);
    agents(AgentList);
    !broadcast_rescue_ask(AgentList, FailedAgent, X, Y);
    .wait(1000);
    ?bids(ResBids);
    select_best_bid(ResBids);
    ?best_bid(Rescuer, X, Y, _Dist);
    .print("Coordinator: rescuer is", Rescuer);
    .send(Rescuer, tell, rescue(GoalId, X, Y));
    !wait_rescue_done(Rescuer, GoalId).

+!broadcast_rescue_ask([Agent|Rest], FailedAgent, GoalId, X, Y) <- Agent \== FailedAgent |
    .send(Agent, tell, rescue_ask(GoalId, X, Y));
    !broadcast_rescue_ask(Rest, FailedAgent, GoalId, X, Y).

+!broadcast_rescue_ask([Agent|Rest], FailedAgent, GoalId, X, Y)
  : Agent == FailedAgent <- 
    !broadcast_rescue_ask(Rest, FailedAgent, GoalId, X, Y).

+!broadcast_rescue_ask([], _, _, _, _) <- true.

+rescue_done(Rescuer, GoalId) <-
    .print("Coordinator: rescue done by", Rescuer);
    .send(Rescuer, tell, resume(GoalId)).