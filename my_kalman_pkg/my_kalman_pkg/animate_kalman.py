import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd


data = pd.read_csv('/home/baris/estimates.csv')
data.columns = data.columns.str.strip()


fig, (ax1, ax2) = plt.subplots(2, 1)
ln1, = ax1.plot([], [], 'b--', label="Estimated Position")
ln2, = ax2.plot([], [], 'b--', label="Estimated Velocity")

ax1.set_xlim(0, len(data))
ax1.set_ylim(min(data['position_est']) - 0.5, max(data['position_est']) + 0.5)
ax2.set_xlim(0, len(data))
ax2.set_ylim(min(data['velocity_est']) - 0.5, max(data['velocity_est']) + 0.5)

ax1.legend()
ax2.legend()
ax1.set_title("Position (Estimated)")
ax2.set_title("Velocity (Estimated)")

def update(frame):
    ln1.set_data(range(frame), data['position_est'][:frame])
    ln2.set_data(range(frame), data['velocity_est'][:frame])
    return ln1, ln2

ani = animation.FuncAnimation(fig, update, frames=len(data), interval=300)


ani.save('kalman_animation.gif', writer='pillow')
