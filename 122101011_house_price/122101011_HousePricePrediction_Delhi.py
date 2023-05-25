#!/usr/bin/env python
# coding: utf-8

# # HOUSE PRICE PREDICTION USING MACHINE LEARNING

# ### Importing Libraries and dataset

# In[14]:


import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# In[15]:


df=pd.read_csv("C:\\Users\\Garima Ranjan\\Downloads\\Delhi.csv")


# ### Data cleaning

# In[16]:


#Since 0 and 1 tells us that the house comes with amenities or not and 9 in the data means not mentioned. So we will convert all 9 to nan and then drop all the missing values
df.replace(9, np.nan, inplace=True)
df.dropna(inplace = True)
df.isna().sum()


# In[17]:


df.shape


# ### Data Processing

# In[18]:


#finding number of objects,integers and floats 
obj=(df.dtypes=='object')
object_cols=list(obj[obj].index)
print("Categorial variables:", len(object_cols))

int_=(df.dtypes=='int64')
int_cols=list(int_[int_].index)
print("Integer variables:",len(int_cols))

fl_=(df.dtypes=='float64')
fl_cols=list(fl_[fl_].index)
print("Float variables:",len(fl_cols))


# In[19]:


df.describe()


# ### Encoding Categorical data

# In[20]:


from sklearn.preprocessing import OneHotEncoder
 
s = (df.dtypes == 'object')
object_cols = list(s[s].index)
print("Categorical variables:")
print(object_cols)
print('No. of. categorical features: ',
      len(object_cols))


# In[21]:


OH_encoder = OneHotEncoder(sparse=False)
OH_cols = pd.DataFrame(OH_encoder.fit_transform(df[object_cols]))
OH_cols.index = df.index
OH_cols.columns = OH_encoder.get_feature_names_out()
df_final = df.drop(object_cols, axis=1)
df_final = pd.concat([df_final, OH_cols], axis=1)


# In[22]:


df_final


# ### Splitting Dataset into training and testing

# In[23]:


from sklearn.metrics import mean_absolute_error
from sklearn.model_selection import train_test_split
 
X = np.array(df_final.drop(['Price'], axis=1))
Y = np.array(df_final['Price'])
Y = Y.reshape(len(Y),1) 
# Split the training set into
# training and validation set
X_train, X_test, Y_train, Y_test = train_test_split(X, Y, train_size=0.8, test_size=0.2, random_state=0)


# ### Using Support Vector Regression(SVR) Model

# In[24]:


from sklearn.metrics import mean_absolute_percentage_error

from sklearn.preprocessing import StandardScaler
sc_X = StandardScaler()
sc_y = StandardScaler()
X_train = sc_X.fit_transform(X_train)
Y_train = sc_y.fit_transform(Y_train)


# In[25]:


from sklearn.svm import SVR
regressor = SVR(kernel = 'rbf')
regressor.fit(X_train, Y_train)


# In[26]:


#predicting the test set results
Y_pred= regressor.predict(X_test)
print(mean_absolute_percentage_error(Y_test, Y_pred))


# In[27]:


Y_pred = sc_y.inverse_transform(regressor.predict(sc_X.transform(X_test)).reshape(-1,1))
np.set_printoptions(precision=2)
print(np.concatenate((Y_pred.reshape(len(Y_pred),1), Y_test.reshape(len(Y_test),1)),1))


# In[28]:


plt.scatter(Y_pred,Y_test)


# ### Using Random Forest Model 

# In[61]:


#resetting values of y_pred and x_pred
X_train, X_test, Y_train, Y_test = train_test_split(X, Y, train_size=0.8, test_size=0.2, random_state=0)


# In[62]:


from sklearn.metrics import mean_absolute_percentage_error
from sklearn.ensemble import RandomForestRegressor
regressor2 = RandomForestRegressor(n_estimators = 500, random_state = 0)
regressor2.fit(X_train, Y_train)


# In[63]:


Y_pred= regressor2.predict(X_test)
print(mean_absolute_percentage_error(Y_test, Y_pred))


# In[64]:


np.set_printoptions(precision=2)
print(np.concatenate((Y_pred.reshape(len(Y_pred),1), Y_test.reshape(len(Y_test),1)),1))


# In[46]:


plt.scatter(Y_pred,Y_test)


# ## Random forest model is best with error of just 0.115

# ## Challenges faced
# 

# In[48]:


#Finding dataset
#Number of data in dataset
#Finding the right model
#increasing accuracy of Random Forest

