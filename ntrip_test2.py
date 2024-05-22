import socket
import base64

def receive_rtcm_data(ntrip_server, port, mountpoint, user, password):
    """NTRIP 서버로부터 RTCM 데이터를 수신하고 출력합니다."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)  # 타임아웃을 10초로 설정

    # 서버에 연결
    sock.connect((ntrip_server, port))

    # NTRIP 요청 보내기
    credentials = f"{user}:{password}"
    encoded_credentials = base64.b64encode(credentials.encode()).decode()
    ntrip_request = (
        f"GET /{mountpoint} HTTP/1.1\r\n"
        f"Host: {ntrip_server}\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "User-Agent: NTRIP PythonClient/1.0\r\n"
        f"Authorization: Basic {encoded_credentials}\r\n"
        "\r\n"
    )
    sock.send(ntrip_request.encode())

    # 서버로부터의 응답 확인 (헤더 읽기)
    response = sock.recv(4096).decode()
    print("Server response headers:\n", response)

    # 데이터 수신 및 출력
    try:
        while True:
            data = sock.recv(4096)
            if data:
                print("Received RTCM Data:", data)
            else:
                print("No more data received.")
                break
    except socket.timeout:
        print("No data received, connection timed out.")
    finally:
        sock.close()

# 사용자 이름, 비밀번호, 서버 정보
ntrip_server = 'www.gnssdata.or.kr'
port = 2101
mountpoint = 'SEOS-RTCM32'
user = 'seollove@gmail.com'
password = 'gnss'

# 데이터 수신 함수 호출
receive_rtcm_data(ntrip_server, port, mountpoint, user, password)
