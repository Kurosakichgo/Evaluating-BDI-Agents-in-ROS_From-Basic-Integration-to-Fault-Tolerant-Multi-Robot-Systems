import asyncio
import websockets
import json

# Example allocation result from auction_allocation function
allocation = {
    'tb1': {'x': -6.0, 'y': -0.5},
    'tb2': {'x': 2.5, 'y': 1.0},
    'tb3': {'x': 5.0, 'y': 1.0}
}

async def send_command(websocket, command):
    await websocket.send(json.dumps(command))

async def send_goal(websocket, turtlebot_name, target):
    goal_msg = {
        "op": "publish",
        "topic": f"/{turtlebot_name}/goal_pose",
        "msg": {
            "header": {
                "frame_id": "map"
            },
            "pose": {
                "position": {
                    "x": target['x'],
                    "y": target['y'],
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1.0
                }
            }
        }
    }
    await send_command(websocket, goal_msg)

async def rosbridge_communication():
    uri = "ws://localhost:9090"  
    
    async with websockets.connect(uri) as websocket:
        for tb, target in allocation.items():
            await send_goal(websocket, tb, target)
            print(f"Sent goal to {tb} for position ({target['x']}, {target['y']})")
            await asyncio.sleep(10)  # Adjust sleep time based on your needs

if __name__ == "__main__":
    asyncio.run(rosbridge_communication())
