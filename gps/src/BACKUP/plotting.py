import numpy as np
import csv
import matplotlib.pyplot as plt

file_list = []
file = open('/home/hyeonseok/TM_result-16.csv', 'r')
read = csv.reader(file)
for f in read:
    file_list.append(f)


npfile = np.asarray(file_list)

x_tm =npfile[:,0]
y_tm =npfile[:,1]
x_kf =npfile[:,2]
y_kf =npfile[:,3]

plt.scatter(x_tm, y_tm, )
plt.show()