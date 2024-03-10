import asyncio
import websockets
from collections import defaultdict
from websockets.exceptions import ConnectionClosedError

connected_clients = defaultdict(set)

async def broadcast_message(path, message):
    if path in connected_clients and connected_clients[path]:
        await asyncio.wait([client.send(message) for client in connected_clients[path]])

async def handler(websocket, path):
    # 클라이언트의 'Authorization' 헤더 확인
    auth_token = websocket.request_headers.get('Sec-WebSocket-Protocol')
    auth_token = auth_token.split(', ') if auth_token else None
    expected_token = 'test_token'  # 조건에 맞는 토큰을 여기에 설정
    print(f"{auth_token=} {expected_token=}")

    if websocket.remote_address[0] == '127.0.0.1':
        # print(f"authentification postponed for internal access : {websocket.remote_address}")
        pass
    else:

        if not auth_token or expected_token not in auth_token:
            print(f"Rejected connection from {websocket.remote_address}: Unauthorized")
            await websocket.close(code=1008, reason="Unauthorized")  # 1008: Policy Violation
            return
    
    print(f"Accepted connection from {websocket.remote_address} on {path}")
    
    connected_clients[path].add(websocket)
    try:
        async for message in websocket:
            print(f"Recv |||  {websocket.remote_address} on {path}: {message}")
            if path == '/agv_1/stat':

                await broadcast_message(path, f"{message}")

            if path == '/agv_1/cmd':
                await broadcast_message(path, f"{message}")
    except ConnectionClosedError:
        print(f"Connection closed by {websocket.remote_address} on {path}")
    finally:
        connected_clients[path].remove(websocket)
        if not connected_clients[path]:
            del connected_clients[path]

async def main():
    async def handshake_handler(websocket, path):
        requested_protocols = websocket.request_headers.get('Sec-WebSocket-Protocol')
        if requested_protocols:
            requested_protocols = requested_protocols.split(',')
            if "test_token" in [protocol.strip() for protocol in requested_protocols]:
                await websocket.accept(subprotocol="test_token")
                print('accepted')
                # 이후 로직 처리
                # async for message in websocket:
                #     await websocket.send(f"Echo: {message}")
            else:
                print("Unsupported protocol. Connection closed.")
                return
        

    
    async with websockets.serve(handler, "0.0.0.0", 8765, ping_interval=None, ping_timeout=30, subprotocols=['test_token']):
        print("Server started at ws://0.0.0.0:8765")
        await asyncio.Future()  

asyncio.run(main())