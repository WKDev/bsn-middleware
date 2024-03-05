import asyncio
import websockets

async def receive_messages():
    async with websockets.connect('ws://localhost:6759') as websocket:
        while True:
            message = await websocket.recv()
            print(f"Received message: {message}")

asyncio.get_event_loop().run_until_complete(receive_messages())