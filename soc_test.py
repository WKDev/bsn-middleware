import socket
import json

# 서버 주소 및 포트
server_address = '192.168.10.2'
server_port = 9760

# JSON 메시지
message = {
"dsID":"www.hc-system.com.RemoteMonitor",
"reqType": "command",
"packID": "0",
# "queryAddr":["version", "curMode"]
"cmdData": ["actionSingleCycle"]
}
stat_req_msg = lambda query_data : {
"dsID":"www.hc-system.com.RemoteMonitor",
"reqType": "query",
"packID": "0",
"queryAddr":query_data
}



# JSON 메시지를 ASCII로 인코딩

stat = {}

# for d in ["axis-n", "curAlarm", "world-n", "isMoving" ]:
cmd_list = ["curMode","curAlarm", "isMoving", "moldList" ] + [f"axis-{x}" for x in range(7)] + [f"world-{x}" for x in range(7)]
message_encoded = json.dumps(stat_req_msg(cmd_list)).encode('ascii')

# 소켓 생성
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    # 서버에 연결
    s.connect((server_address, server_port))
    
    # 메시지 전송
    s.sendall(message_encoded)
    
    print(f"sending{ message_encoded}")
    
    # 서버로부터의 응답 받기
    resp_buf = s.recv(1024).decode("ascii")

    # Convert response into dictionary
    resp = json.loads(resp_buf)

    for k, v in zip(resp['queryAddr'], resp['queryData']):
        if k == "curMode":
            #0: None, 1: Manual Mode, 2: Automatic Mode, 3: Stop Mode, 7: Auto-running, 8:Step-by-Step, 9: Single Loop
            mode_list = ["none", "Manual Mode", "Automatic Mode", "Stop Mode", "Auto-running", "Step-by-Step", "Single Loop"]
            stat[k] = mode_list[int(v)]

        elif k == "isMoving":
            stat[k] = "Moving" if v == "1" else "Stopped"
        
        else:
            stat[k] = round(float(v),3)

        # print(f"{k} : {stat[k]}")

        stat_json = json.dumps(stat)

    print (stat_json)



