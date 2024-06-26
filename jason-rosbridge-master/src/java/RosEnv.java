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

    private Logger logger = Logger.getLogger("hello_ros."+RosEnv.class.getName());
	private Publisher cmdVelPublisher;

    
    RosBridge bridge = new RosBridge();

    /** Called before the MAS execution with the args informed in .mas2j */
    @Override
    public void init(String[] args) {
        super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");
		cmdVelPublisher = new Publisher("/cmd_vel", "geometry_msgs/Twist", bridge);

		
		bridge.subscribe(SubscriptionRequestMsg.generate("/ros_to_java")
				.setType("std_msgs/String")
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {

				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(PrimitiveMsg.class);
					PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
					logger.info(msg.data);
				}
			}
	);
    }

    @Override
    public boolean executeAction(String agName, Structure action) {
		if (action.getFunctor().equals("move_forward")) {
			move_forward();
		}
		else if (action.getFunctor().equals("hello_ros")){
			hello_ros();
			
		}
		else if (action.getFunctor().equals("move_left")){
			move_left();
			
		}
		else if (action.getFunctor().equals("move_right")){
			move_right();
			
		}
		else if (action.getFunctor().equals("stop")){
			stopping();
			
		}
        informAgsEnvironmentChanged();
        return true; // the action was executed with success
    }
    
	
	public void move_forward(){
        
		Publisher cmdVelPublisher = new Publisher("/cmd_vel", "geometry_msgs/Twist", bridge);

        Vector3 linear = new Vector3(0.4, 0, 0); //
        Vector3 angular = new Vector3(0, 0, 0);
        cmdVelPublisher.publish(new Twist(linear, angular));
        

	}
    
	public void move_left(){
        
		Publisher cmdVelPublisher = new Publisher("/cmd_vel", "geometry_msgs/Twist", bridge);

        Vector3 linear = new Vector3(0, 0, 0); // 0.2 m/s
        Vector3 angular = new Vector3(0, 0, 0.5);
        cmdVelPublisher.publish(new Twist(linear, angular));
        

	}

	public void move_right(){
        
		Publisher cmdVelPublisher = new Publisher("/cmd_vel", "geometry_msgs/Twist", bridge);

        Vector3 linear = new Vector3(0, 0, 0); // 0.2 m/s
        Vector3 angular = new Vector3(0, 0, -0.5);
        cmdVelPublisher.publish(new Twist(linear, angular));
        

	}
    
    public void stopping(){
        
		Publisher cmdVelPublisher = new Publisher("/cmd_vel", "geometry_msgs/Twist", bridge);

        Vector3 linear = new Vector3(0, 0, 0); // 0.2 m/s
        Vector3 angular = new Vector3(0, 0, 0);
        cmdVelPublisher.publish(new Twist(linear, angular));
        

	}

    public void hello_ros() {
		
		Publisher pub = new Publisher("/java_to_ros", "std_msgs/String", bridge);
		
		for(int i = 0; i < 100; i++) {
			pub.publish(new PrimitiveMsg<String>("hello from Jason " + i));
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
	}

    /** Called before the end of MAS execution */
    @Override
    public void stop() {
        super.stop();
    }
}