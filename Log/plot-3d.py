import pandas as pd 
import matplotlib.pyplot as plt 
import seaborn as sns; sns.set_style('darkgrid')
from mpl_toolkits.mplot3d import Axes3D 

df = pd.read_csv('mag_test_no-motor.csv', header=None)

fig = plt.figure()
ax = Axes3D(fig)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.plot(df[0], df[1], df[2], marker="o", linestyle='None')

plt.show()