import asyncio
import websockets

async def receive_messages():
    i = 0
    while True:
        async with websockets.connect('ws://localhost:8765/agv_1/cmd') as websocket:
            i = i + 1
            print(f"Sending {i}")
            await websocket.send(f"Sending {i}")
            # message = await websocket.recv()
            await asyncio.sleep(0.1)


asyncio.get_event_loop().run_until_complete(receive_messages())