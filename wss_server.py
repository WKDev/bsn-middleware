import asyncio
import websockets

try:
    async def echo(websocket, path):
        async for message in websocket:
            print(f"Received message: {message}")
            await websocket.send(f"Echo: {message}")

    start_server = websockets.serve(echo, "0.0.0.0", 8765)

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()

except websockets.exceptions.ConnectionClosedOK:
    pass
