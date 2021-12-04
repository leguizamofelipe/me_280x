import numpy as np
import trouve as tv
import matplotlib.pyplot as plt

dataset_size = 50
distance_threshold = 10

input_data = np.zeros(dataset_size)
input_data = np.array([3,5,3,4,2,4,5,4,45,43,45,3,4,4,4,2,2,5, np.inf, np.inf, np.inf, np.inf,4,2,2,3,5,13,43,12,3,2,3,2])

pass_zones = tv.find_events(input_data > distance_threshold, period = 1)
blocked_zones = tv.find_events(input_data <= distance_threshold, period = 1)

for event in pass_zones:
    plt.plot([event.start, event.stop], [0,0])

for event in blocked_zones:
    plt.plot([event.start, event.stop], [1,1])

centers = []

for zone in pass_zones:
    center = (zone.stop + zone.start) / 2
    centers.append(center)

plt.show()

print('done')
