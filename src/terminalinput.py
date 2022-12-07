import numpy as np

# i=0
# array = np.zeros((5,3))

# # print(array)
# # print(array[1,2])
# some =True

# while some==True:
#     print(i) 
#     array[i] = [0,1,0]
#     print(array)
#     i+= 1 

#     dist1 =np.linalg.norm(array[0]-array[1])
#     dist2 =np.linalg.norm(array[1]-array[2])
#     dist3 =np.linalg.norm(array[2]-array[3])
#     dist4 =np.linalg.norm(array[4]-array[0])

#     dist_ave = (dist1 +dist2 + dist3 + dist4)/4
#     print(dist_ave)
#     if i == array.shape[0]:
#         i=0
#         some=False
    

# from matplotlib import pyplot as plt




# import matplotlib.pyplot as plt
# import numpy as np

# x= [1,2,3,4,5,6]
# y= [1,2,3,4,5,6]



# plt.scatter(x,y)


# x = np.linspace(0, 10, 30)
# y = np.sin(x)

# plt.plot(x, y, 'o', color='black')


import matplotlib.pyplot as plt
import numpy as np

# X axis parameter:
xaxis = np.array([2, 8])

# Y axis parameter:
yaxis = np.array([4, 9])

plt.plot(xaxis, yaxis)
plt.show()

