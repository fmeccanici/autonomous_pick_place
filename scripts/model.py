import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import ast
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score
import pickle
from joblib import dump, load

model = load('linear_regressor_angle.joblib')

# print(model.coef_*np.deg2rad(90+-75) + model.intercept_)
print(model.coef_)
print(model.intercept_)

# print(model.predict([[np.deg2rad(90 + -75)]]))
print(model.coef_*np.deg2rad(90+-75)+ model.intercept_)