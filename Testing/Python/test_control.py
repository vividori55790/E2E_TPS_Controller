import serial
import struct
import time

# === 설정 영역 ===
PORT = 'COM6'  # 중요: 장치 관리자에서 확인한 NUCLEO 포트 번호로 수정 (예: COM3, COM4)
BAUD = 115200  # STM32 설정과 동일하게 유지


def main():
    try:
        # 시리얼 포트 열기
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"✅ {PORT} 포트에 성공적으로 연결되었습니다.")

        while True:
            print("-" * 30)
            try:
                # 사용자 입력 받기
                v_target = float(input("1. 목표 전압(V) 입력: "))
                i_limit = float(input("2. 전류 제한(A) 입력: "))
                b_type = int(input("3. 배터리 타입(1:Li-ion, 2:LFP, 3:LTO): "))

                # 데이터 패킹 (STM32의 DAB_Packet 구조체와 일치해야 함)
                # < : Little-endian (STM32 기본)
                # B : uint8 (1 byte) - Start Byte (0xAA)
                # f : float (4 bytes) - Target Voltage
                # f : float (4 bytes) - Current Limit
                # B : uint8 (1 byte) - Battery Type
                # B : uint8 (1 byte) - End Byte (0x55)
                packet = struct.pack('<BffBB', 0xAA, v_target, i_limit, b_type, 0x55)

                # 데이터 전송
                ser.write(packet)
                print(f"▶ 데이터 전송 완료: V={v_target}, I={i_limit}, Type={b_type}")

                # STM32의 응답 대기 ("OK\n" 확인)
                response = ser.readline().decode('utf-8').strip()
                if response == "OK":
                    print(f"◀ STM32 응답: {response} (수신 확인됨)")
                else:
                    print(f"⚠️ 응답 없음 또는 오류: {response}")

            except ValueError:
                print("❌ 숫자만 입력해 주세요.")

            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"❌ 포트를 열 수 없습니다: {e}")
    except KeyboardInterrupt:
        print("\n⏹ 프로그램을 종료합니다.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
