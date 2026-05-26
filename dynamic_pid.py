import matplotlib.pyplot as plt
import numpy as np

error_x = np.linspace(-120, 120, 500)
error_y = np.linspace(-120, 120, 500)

ki = 0.7 * np.tanh(np.abs(error_x) / 65) + 0.3

kp = 0.7 * np.tanh(np.abs(error_y) / 60) + 0.3

plt.xlabel('error')
plt.ylabel('ratio')
plt.plot(error_x, ki, error_y, kp)
plt.show()