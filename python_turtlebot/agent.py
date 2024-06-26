import asyncio
import websockets
import json

async def send_command(websocket, command):
    await websocket.send(json.dumps(command))

async def rosbridge_communication():
    uri = "ws://localhost:9090"  
    
    async with websockets.connect(uri) as websocket:
        
        await send_command(websocket, {
            "op": "advertise",
            "topic": "/cmd_vel",
            "type": "geometry_msgs/Twist"
        })
        
       
        move_command = {
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear": {"x": 0.4, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
        }
        
        
        await send_command(websocket, move_command)
        
        
        await asyncio.sleep(8.7)
        
        # 停止移动（发送零速度）
        stop_command = {
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
        }
        
        await send_command(websocket, stop_command)

        left_command = {
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.5}
            }
           
            }
        
        right_command = {
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": -0.5}
            }
         }

       

        await send_command(websocket, left_command)

        await asyncio.sleep(3.21)

        await send_command(websocket, stop_command)

        await send_command(websocket, move_command)

        await asyncio.sleep(2.7)
        
        await send_command(websocket, stop_command)
        
        await send_command(websocket, left_command)
        
        await asyncio.sleep(3.16)
        
        await send_command(websocket, stop_command)

        await send_command(websocket, move_command)

        await asyncio.sleep(8.4)

        await send_command(websocket, stop_command)

        await send_command(websocket, right_command)

        await asyncio.sleep(3.23) 
        
        await send_command(websocket, stop_command)
 
        await send_command(websocket, move_command)

        await asyncio.sleep(2.7)

        await send_command(websocket, right_command)

        await asyncio.sleep(3.2) 
        
        await send_command(websocket, stop_command)
 
        await send_command(websocket, move_command)

        await asyncio.sleep(8.7)

        await send_command(websocket, stop_command)






 
   





if __name__ == "__main__":
    asyncio.run(rosbridge_communication())
