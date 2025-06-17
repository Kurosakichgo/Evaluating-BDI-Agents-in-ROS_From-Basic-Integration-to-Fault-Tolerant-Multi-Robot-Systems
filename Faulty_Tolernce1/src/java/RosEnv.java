package src.java;
import jason.asSyntax.*;
import jason.environment.*;
import java.util.logging.*;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;
import com.fasterxml.jackson.databind.JsonNode;
import ros.msgs.geometry_msgs.Vector3;
import ros.msgs.geometry_msgs.Twist;

public class RosEnv extends Environment {

    private Logger logger = Logger.getLogger("hello_ros." + RosEnv.class.getName());
    private Publisher cmdVelPublisher1;
    private Publisher cmdVelPublisher2;
    private Publisher cmdVelPublisher3;

    private RosBridge bridge;

    @Override
    public void init(String[] args) {
        super.init(args);
        bridge = new RosBridge();
        bridge.connect("ws://localhost:9090", true);
        logger.info("Environment started, connection with ROS established.");

        // Initialize publishers for different robots
        cmdVelPublisher1 = new Publisher("/tb1/cmd_vel", "geometry_msgs/Twist", bridge);
        cmdVelPublisher2 = new Publisher("/tb2/cmd_vel", "geometry_msgs/Twist", bridge);
        cmdVelPublisher3 = new Publisher("/tb3/cmd_vel", "geometry_msgs/Twist", bridge);

        // Subscribe to topics to receive information from the robots (e.g., obstacle detection)
        subscribeToRobot("/tb1/obstacle", "tb1");
        subscribeToRobot("/tb2/obstacle", "tb2");
        subscribeToRobot("/tb3/obstacle", "tb3");
    }

    private void subscribeToRobot(String topic, String agName) {
        bridge.subscribe(SubscriptionRequestMsg.generate(topic)
                .setType("std_msgs/String")
                .setThrottleRate(1)
                .setQueueLength(1),
            new RosListenDelegate() {
                public void receive(JsonNode data, String stringRep) {
                    MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<>(PrimitiveMsg.class);
                    PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
                    logger.info("Received message from " + topic + ": " + msg.data);
                    handleObstacleMessage(agName, msg.data);
                }
            }
        );
    }

    private void handleObstacleMessage(String agName, String data) {
        addPercept(agName, Literal.parseLiteral("obstacle_detected"));
    }

    @Override
    public boolean executeAction(String agName, Structure action) {
        switch (action.getFunctor()) {
            case "move_forward":
                move_forward(agName);
                break;
            case "move_left":
                move_left(agName);
                break;
            case "move_right":
                move_right(agName);
                break;
            case "stop":
                stopping(agName);
                break;
            case "monitor":
                String location = ((StringTerm) action.getTerm(0)).getString();
                monitor(agName, location);
                break;
            case "patrol":
                String patrolLocation = ((StringTerm) action.getTerm(0)).getString();
                patrol(agName, patrolLocation);
                break;
            case "avoid_obstacle":
                avoid_obstacle(agName);
                break;
            default:
                logger.warning("Unknown action: " + action.getFunctor());
                return false;
        }

        informAgsEnvironmentChanged();
        return true; // the action was executed with success
    }

    public void patrol(String agName, String location) {
        logger.info(agName + " patrolling " + location);
        move_to(agName, location);
    }

    public void avoid_obstacle(String agName) {
        logger.info(agName + " avoiding obstacle");
        move_left(agName); // Simple example of avoidance, should be improved
    }

    public void move_to(String agName, String location) {
        // 根据location实现移动逻辑，这里使用示例的逻辑
        logger.info(agName + " moving to " + location);
        move_forward(agName);
    }

    public void monitor(String agName, String location) {
        // 实现监控逻辑
        logger.info(agName + " monitoring " + location);
    }

    private void move_forward(String agName) {
        Publisher cmdVelPublisher = getPublisherForAgent(agName);
        Vector3 linear = new Vector3(0.4, 0, 0);
        Vector3 angular = new Vector3(0, 0, 0);
        cmdVelPublisher.publish(new Twist(linear, angular));
    }

    private void move_left(String agName) {
        Publisher cmdVelPublisher = getPublisherForAgent(agName);
        Vector3 linear = new Vector3(0, 0, 0);
        Vector3 angular = new Vector3(0, 0, 0.5);
        cmdVelPublisher.publish(new Twist(linear, angular));
    }

    private void move_right(String agName) {
        Publisher cmdVelPublisher = getPublisherForAgent(agName);
        Vector3 linear = new Vector3(0, 0, 0);
        Vector3 angular = new Vector3(0, 0, -0.5);
        cmdVelPublisher.publish(new Twist(linear, angular));
    }

    private void stopping(String agName) {
        Publisher cmdVelPublisher = getPublisherForAgent(agName);
        Vector3 linear = new Vector3(0, 0, 0);
        Vector3 angular = new Vector3(0, 0, 0);
        cmdVelPublisher.publish(new Twist(linear, angular));
    }

    private Publisher getPublisherForAgent(String agName) {
        switch (agName) {
            case "tb1":
                return cmdVelPublisher1;
            case "tb2":
                return cmdVelPublisher2;
            case "tb3":
                return cmdVelPublisher3;
            default:
                throw new IllegalArgumentException("Unknown agent name: " + agName);
        }
    }

    @Override
    public void stop() {
        
    }
}
