import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import ast
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score


head_joints = pd.read_csv('extra_joints_slash_state.csv')
markers = pd.read_csv('marker_detector_slash_markers.csv')
tf = pd.read_csv('tf_base_xtion.csv')

rounded_timestamps_markers = map(lambda x: round(x/float((10**(18))),9),  markers['rosbagTimestamp'])
markers['rounded_timestamps'] = pd.Series(rounded_timestamps_markers)
markers = markers.drop_duplicates(subset='rounded_timestamps', keep='first')

rounded_timestamps_tf = map(lambda x: round(x/float((10**(18))),9),  tf['rosbagTimestamp'])
tf['rounded_timestamps'] = pd.Series(rounded_timestamps_tf)
tf = tf.drop_duplicates(subset='rounded_timestamps', keep='first')






angle = np.array((map(lambda x : ast.literal_eval(ast.literal_eval(x))[0], tf['data'])))
z = np.array(markers[['y']])

# print(angle)
# linear_regressor = linear_model.LinearRegression()
# linear_regressor.fit([angle],z)

# y_pred = linear_regressor.predict([angle])



plt.scatter(angle,z)
# plt.plot(angle,y_pred)
plt.grid()
plt.show()