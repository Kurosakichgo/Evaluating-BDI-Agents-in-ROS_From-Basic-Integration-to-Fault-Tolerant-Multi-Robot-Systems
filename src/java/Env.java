package src.java;

import jason.asSyntax.*;
import jason.environment.*;
import java.util.*;
import java.util.logging.*;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;
import com.fasterxml.jackson.databind.JsonNode;



public class Env extends Environment {

    private Logger logger = Logger.getLogger("hello_ros." + Env.class.getName());
    private RosBridge bridge = new RosBridge();
    private List<Literal> bids = new ArrayList<>();

    @Override
    public void init(String[] args) {
        super.init(args);
        bridge.connect("ws://localhost:9090", true);
        logger.info("Environment started, connection with ROS established.");
        
        bridge.subscribe(SubscriptionRequestMsg.generate("/ros_to_java")
                .setType("std_msgs/String")
                .setThrottleRate(1)
                .setQueueLength(1),
            new RosListenDelegate() {
                public void receive(JsonNode data, String stringRep) {
                    MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<>(PrimitiveMsg.class);
                    PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
                    logger.info(msg.data);
                }
            }
        );
    }

    @Override
    public boolean executeAction(String agName, Structure action) {
        switch (action.getFunctor()) {
            case "select_best_bid":
                return select_best_bid(action);
            case "calculate_distance_for_bid":
                return calculate_distance_for_bid(agName, action);
            case "append":
                return append(agName, action);
            case "add_bid":
                return add_bid(action);
            case "send_goal":
                return send_goal(action);
            default:
                logger.warning("Unknown action: " + action.getFunctor());
                return false;
        }
    }

    public boolean select_best_bid(Structure action) {
        try {
            ListTerm bidsList = (ListTerm) action.getTerm(0);
            if (bidsList.size() == 0) {
                logger.warning("No bids received.");
                return false;
            }
    
            // 存储每个 GoalId 最佳的 bid
            Map<Term, Structure> bestBids = new HashMap<>();
            Map<Term, Double> bestValues = new HashMap<>();
    
            // 遍历所有 bids
            for (Term bidTerm : bidsList) {
                Structure bid = (Structure) bidTerm;
                Term goalId = bid.getTerm(1);
                double bidValue = ((NumberTerm) bid.getTerm(4)).solve();
    
                // 如果该 GoalId 没有记录，或者当前 bid 更优，则更新
                if (!bestBids.containsKey(goalId) || bidValue < bestValues.get(goalId)) {
                    bestBids.put(goalId, bid);
                    bestValues.put(goalId, bidValue);
                }
            }
    
            // 将最佳的 bid 作为感知加入
            clearPercepts();
            for (Map.Entry<Term, Structure> entry : bestBids.entrySet()) {
                Structure bestBid = entry.getValue();
                Term bestAgent = bestBid.getTerm(0);
                Term goalId = bestBid.getTerm(1);
                Term gx = bestBid.getTerm(2);
                Term gy = bestBid.getTerm(3);
                Term bestDistance = bestBid.getTerm(4);
                Literal bestBidLiteral = ASSyntax.createLiteral("best_bid", bestAgent, goalId, gx, gy, bestDistance);
                addPercept(bestBidLiteral);
            }
    
            bids.clear(); // 清空 bids 列表
            return true;
        } catch (Exception e) {
            logger.warning("Error selecting best bid: " + e.getMessage());
            return false;
        }
    }
    
    
   

    public boolean calculate_distance_for_bid(String agName, Structure action) {
        try {
           
            double x = ((NumberTerm) action.getTerm(0)).solve();
            double y = ((NumberTerm) action.getTerm(1)).solve();
            double gx = ((NumberTerm) action.getTerm(2)).solve();
            double gy = ((NumberTerm) action.getTerm(3)).solve();
            String goalId = ((Atom) action.getTerm(5)).toString();

            double distance = calculateManhattanDistance(x, y, gx, gy);
            String agent = ((Atom) action.getTerm(4)).toString();
            logger.info("Calculated distance: " + distance + " for goal (" + gx + ", " + gy + ")");
            addPercept(agName, Literal.parseLiteral("distance(" + goalId + "," + agent + ", " + gx + ", " + gy + ", " + distance + ")"));
            return true;
        } catch (Exception e) {
            logger.warning("Error calculating distance: " + e.getMessage());
            return false;
        }
    }

    private double calculateManhattanDistance(double x, double y, double gx, double gy) {
        return Math.abs(gx - x) + Math.abs(gy - y);
    }
    
    public boolean append(String agName, Structure action) {
        try {
            Literal bid = (Literal) action.getTerm(0);
            bids.add(bid);
            return true;
        } catch (Exception e) {
            logger.warning("Error appending bid: " + e.getMessage());
            return false;
        }
    }
    

    public boolean add_bid(Structure action) {
        try {
            Literal bid = ASSyntax.createLiteral("bid", action.getTermsArray());
            bids.add(bid);
            logger.info("Stored bid: " + bid);
            return true;
        } catch (Exception e) {
            logger.warning("Error storing bid: " + e.getMessage());
            return false;
        }
    }

   public boolean send_goal(Structure action) {
    try {
        double gx = ((NumberTerm) action.getTerm(0)).solve();
        double gy = ((NumberTerm) action.getTerm(1)).solve();
        String turtlebotName = ((StringTerm) action.getTerm(2)).getString();

        long secs = System.currentTimeMillis() / 1000;

        // 构建时间戳
        Map<String, Object> stamp = new HashMap<>();
        stamp.put("secs", secs);
        stamp.put("nsecs", 0);

        // 构建 header
        Map<String, Object> header = new HashMap<>();
        header.put("frame_id", "map");
        header.put("stamp", stamp);

        // 构建 position
        Map<String, Object> position = new HashMap<>();
        position.put("x", gx);
        position.put("y", gy);
        position.put("z", 0.0);

        // 构建 orientation
        Map<String, Object> orientation = new HashMap<>();
        orientation.put("w", 1.0);

        // 构建 pose
        Map<String, Object> pose = new HashMap<>();
        pose.put("position", position);
        pose.put("orientation", orientation);

        // 构建总消息体
        Map<String, Object> goalMessage = new HashMap<>();
        goalMessage.put("header", header);
        goalMessage.put("pose", pose);

        // 发送给 rosbridge
        bridge.publish("/" + turtlebotName + "/goal_pose", "geometry_msgs/PoseStamped", goalMessage);

        logger.info("Goal sent to " + turtlebotName + ": (" + gx + ", " + gy + ")");
        return true;

    } catch (Exception e) {
        logger.warning("Error sending goal: " + e.getMessage());
        return false;
    }
}

    @Override
    public void stop() {
        super.stop();
    }
}
