from functions import *

st_orient = IMUOrientation()
     
for d in range(3):
    GrandChallenge(st_orient)

plt.plot(xlist,ylist)
plt.savefig('Path.png')
plt.show()