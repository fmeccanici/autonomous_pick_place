import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import ast
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score
import pickle

# linear_regressor = linear_model.LinearRegression()
# linear_regressor.fit(X,y)

# y_pred = linear_regressor.predict(X)
filename = 'linear_regressor_angle.sav'
linear_regressor = pickle.load(open(filename, 'rb'))


head_joints = pd.read_csv('extra_joints_slash_state.csv')
markers = pd.read_csv('marker_detector_slash_markers.csv')
tf = pd.read_csv('tf_base_xtion.csv')

rounded_timestamps_markers = map(lambda x: round(x/float((10**(18))),9),  markers['rosbagTimestamp'])
markers['rounded_timestamps'] = pd.Series(rounded_timestamps_markers)
markers = markers.drop_duplicates(subset='rounded_timestamps', keep='first')

rounded_timestamps_tf = map(lambda x: round(x/float((10**(18))),9),  tf['rosbagTimestamp'])
tf['rounded_timestamps'] = pd.Series(rounded_timestamps_tf)
tf = tf.drop_duplicates(subset='rounded_timestamps', keep='first')


# rounded_timestamps_tf = map(lambda x: round(x/float((10**(18))),9),  tf['rosbagTimestamp'])
# tf['rounded_timestamps'] = pd.Series(rounded_timestamps_tf)
# tf = tf.drop_duplicates(subset='rounded_timestamps', keep='first')



angle = np.array((map(lambda x : ast.literal_eval(ast.literal_eval(x))[0], tf['data'])))

theta = angle + 90
theta_rad = np.deg2rad(theta)


z = np.array(markers[['y']])
x = np.array(markers[['position']])
z = np.array(map(lambda x: float(x),z))
x = np.array(map(lambda x: float(x),x))


print(len(x))
# linear_regressor.fit(x.reshape(-1,1), z.reshape(-1,1))
z_lin = linear_regressor.predict((np.tan(theta_rad)*x).reshape(-1,1))


print(len(z_lin.reshape(-1,1)))

plt.plot(x.reshape(-1,1), z_lin.reshape(-1,1), color='red')


plt.scatter(x,z)
plt.grid()
plt.xlabel('x coordinate of marker expressed in base_footprint [m]')
plt.ylabel('z coordinate of marker expressed in base_footprint [m]')
plt.title('Offset visualization')
plt.show()