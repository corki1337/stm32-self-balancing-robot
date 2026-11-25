import serial
from datetime import datetime
from pathlib import Path

PORT = "COM3"
BAUD = 921600



log_dir = Path(__file__).parent.parent / "logs"
log_file = log_dir / f"{datetime.now():%Y%m%d_%H%M%S}.csv"

log_dir.mkdir(exist_ok=True)


with serial.Serial(port=PORT, baudrate=BAUD, timeout=1) as ser, open(log_file, "w") as f :

    f.write("tick,complementary_angle,kalman_angle,angle_setpoint,pid_output,wheel_speed\n")

    ser.reset_input_buffer()
    i = 0
    while True:

        try:

            line = ser.readline()
            pola = line.decode(errors="ignore").strip().split(",")
            if(len(pola) != 6):
                continue
            f.write(",".join(pola) + "\n")
            i += 1
        except KeyboardInterrupt:
            print(f"Saved {i} lines in {log_file}")
            break
        

