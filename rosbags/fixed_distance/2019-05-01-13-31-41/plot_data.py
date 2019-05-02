import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import ast
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score
import pickle

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


zero_point = float(markers[map(lambda x: round(x,2)==-90, angle)]['y'])


# angle_offset = angle + 5

z = np.array(markers[['y']])
x = np.array(markers[['position']])

z = np.array(map(lambda x: float(x),z))
x = np.array(map(lambda x: float(x),x))

# print(x)
# print(angle)
# linear_regressor = linear_model.LinearRegression()
# linear_regressor.fit([angle],z)

# y_pred = linear_regressor.predict([angle])

theta = angle + 90
# theta_offset = angle_offset + 90

theta_offset = 5

theta_rad = np.deg2rad(theta)
theta_offset_rad = np.deg2rad(theta_offset)

# print(90+angle)

# z_expected = np.array(map(lambda x,theta : (np.tan(theta)*np.mean(x)), angle_rad, x))

# print(len(x))


z2 = np.tan(theta_rad)*(x)
z3 = np.tan(theta_rad + theta_offset_rad)*(x) - np.tan(theta_rad)*(x)

# z2 = np.tan(theta_rad)*np.mean(x)
# z3 = np.tan(theta_rad + theta_offset_rad)*np.mean(x) - np.tan(theta_rad)*np.mean(x)


# print(list(map(lambda x: isinstance(x,string), x)))

# z_true = 1.16 # true marker height wrt base
# z_true = zero_point
z_true = 1.16
# print(z_true)
# print(z3)
# print(z_true - z3)
# print('angle size: ' + str(len(angle)) + ' z size: ' + str(len(z_true - z3)))
# z_expected = np.tan(angle_rad)*np.mean(x) + 1.13
# z_expected2 = np.tan(angle_rad2)*np.mean(x) + 1.13

# print(z_expected2)

# print(theta)
# plt.scatter(angle,z)
# plt.scatter(angle,x)

# print(z3)

linear_regressor = linear_model.LinearRegression()

# print(([angle]))
# print((z))
# angle.reshape(len(angle),1)

linear_regressor.fit((np.tan(theta_rad)*x).reshape(-1,1),z.reshape(-1,1))

filename = 'linear_regressor_angle.sav'
pickle.dump(linear_regressor, open(filename, 'wb'))

z_lin = linear_regressor.predict((np.tan(theta_rad)*x).reshape(-1,1))


print(len(z_lin))

angle = angle.reshape(1,193)
z_lin = z_lin.reshape(1,193)

# plt.scatter(angle, z_true - (z3-0.12) )
# print(angle)

# z_offset = z_true - z_lin

plt.scatter(angle,z)

# print(linear_regressor.coef_)
z_offset = z - (z_lin - z_true)

# plt.scatter(angle, z_offset)



plt.plot(angle.reshape(-1,1), z_lin.reshape(-1,1),color='red')
plt.grid()
plt.xlabel('angle of head joint expressed in base_footprint [deg]')
plt.ylabel('z coordinate of marker expressed in base_footprint [m]')
plt.title('Offset visualization')

plt.show()

# plt.scatter(angle, z_true + (z3-0.12) )

# plt.plot()

# plt.scatter(angle,z)
# plt.grid()
# plt.xlabel('angle of head joint expressed in base_footprint [deg]')
# plt.ylabel('z coordinate of marker expressed in base_footprint [m]')
# plt.title('Offset visualization')
# plt.show()
# plt.scatter(angle, (z_expected-z_expected2))
# plt.grid()
# 