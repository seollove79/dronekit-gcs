import socket
import base64
from dronekit import connect

def get_rtcm_data(ntrip_server, port, mountpoint, user, password):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(30)  # 타임아웃을 30초로 조정
    sock.connect((ntrip_server, port))

    ntrip_request = f'GET /{mountpoint} HTTP/1.1\r\n'
    ntrip_request += f'Host: {ntrip_server}\r\n'
    ntrip_request += 'Ntrip-Version: Ntrip/2.0\r\n'
    ntrip_request += 'User-Agent: NTRIP PythonClient/1.0\r\n'
    ntrip_request += f'Authorization: Basic {base64.b64encode(f"{user}:{password}".encode()).decode()}\r\n'
    ntrip_request += '\r\n'
    sock.send(ntrip_request.encode())

    # 헤더를 읽고 버리기
    response = sock.recv(4096)
    headers, _, body = response.partition(b'\r\n\r\n')
    print("Headers Received:", headers.decode())

    if body:
        yield body  # 초기 바디 처리

    while True:
        try:
            data = sock.recv(4096)
            if data:
                print("Received RTCM Data")
                yield data
            else:
                print("No data received, retrying...")
        except socket.timeout:
            print("No data received, retrying...")
        except Exception as e:
            print(f"An error occurred: {e}")
            break


def send_rtcm_to_pixhawk(vehicle, rtcm_data):
    msglen = 180
    total_length = len(rtcm_data)
    inject_seq_nr = 0

    for start in range(0, total_length, msglen):
        end = min(start + msglen, total_length)
        datachunk = rtcm_data[start:end]
        flags = 0b000  # 기본 플래그 설정 (분할 없음)
        if total_length > msglen:
            flags = 0b001  # 분할 플래그 설정
            flags |= (inject_seq_nr & 0x1f) << 3
        vehicle.message_factory.gps_rtcm_data_send(
            flags,
            len(datachunk),
            bytearray(datachunk.ljust(msglen, b'\0'))
        )
        inject_seq_nr += 1


def main():
    """메인 함수에서는 드론에 연결하고 RTCM 데이터를 처리합니다."""

    # 드론 연결 정보
    connect_type = "tcp"
    ip_addr = "192.168.144.101"
    connect_port = 14550
    connect_string = f"{connect_type}:{ip_addr}:{connect_port}"

    try:
        vehicle = connect(connect_string, wait_ready=True, timeout=60)
        print("Connected to vehicle.")
    except Exception as e:
        print(f"Failed to connect to vehicle: {e}")
        return

    # RTCM 스트림 사용 및 드론에 데이터 전송

    ntrip_server = 'www.gnssdata.or.kr'
    port = 2101
    mountpoint = 'SEOS-RTCM32'
    user = 'seollove@gmail.com'
    password = 'gnss'

    # url = 'RTS2.ngii.go.kr'
    # port = 2101
    # mountpoint = 'VRS-RTCM23'
    # user = 'seollove79'
    # password = 'ngii'

    for rtcm_message in get_rtcm_data(ntrip_server, port, mountpoint, user, password):
        send_rtcm_to_pixhawk(vehicle, rtcm_message)

if __name__ == "__main__":
    main()
