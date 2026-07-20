"""
串口监视器 - 读取 STM32 调试输出 (USART1, 115200bps)
用法: python tools/monitor.py [波特率]
"""
import serial
import serial.tools.list_ports
import sys
import time

PORT = "COM10"
BAUD = 115200

def main():
    baud = int(sys.argv[1]) if len(sys.argv) > 1 else BAUD

    try:
        ser = serial.Serial(PORT, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERR] 无法打开 {PORT}: {e}")
        print("可用端口:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}")
        sys.exit(1)

    print(f"[OK] 已连接 {PORT} @ {baud} bps")
    print(f"[OK] 按 Ctrl+C 退出\n")

    try:
        buf = b""
        while True:
            data = ser.read(256)
            if data:
                # 打印可读内容，替换非ASCII
                text = data.decode("utf-8", errors="replace")
                print(text, end="", flush=True)

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[BYE] 退出")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
