from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

r = 0.0 / 180.0 * math.pi
p = 0.0 / 180.0 * math.pi
y = 700.0 / 180.0 * math.pi

print("r: {} p: {} y: {}".format(r,p,y))
q = quaternion_from_euler(r, p, y)

e = euler_from_quaternion(q)

theta = e[2]
print("theta: ", theta)
if (theta < 0):
    theta += 2*math.pi

print(e[0]/math.pi*180, e[1]/math.pi*180, theta)
