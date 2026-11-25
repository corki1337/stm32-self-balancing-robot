import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path

log_dir = Path(__file__).parent.parent / "logs"
files = log_dir.glob("*.csv")
log_file = sorted(files)[-1]

df = pd.read_csv(log_file)

t = (df["tick"] - df["tick"].iloc[0]) / 1000

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

ax1.plot(t, df["complementary_angle"], label="angle")
ax1.plot(t, df["angle_setpoint"], label="setpoint")
ax1.set_ylabel("angle [°]")
ax1.legend()
ax1.grid(True)


ax2.set_xlabel("time [s]")
ax2.plot(t, df["pid_output"], label = "pid output")
ax2.set_ylabel("motor PWM output [%]")
ax2.legend()
ax2.grid(True)
plt.show()




