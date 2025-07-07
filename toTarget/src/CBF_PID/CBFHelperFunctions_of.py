import math

  

#Notes: 

# Note 1: requires install of qpsolvers, commands to run in terminal/powershell found below:
'''
pip install qpsolvers
''' 

#Note 2:
#Grad(h, x) untested as I couldn't get numdifftools import to be recognized, even after intsalling the package.
#Could be a problem with how I installed or IDE, good chance we will have to write our own Grad(h, x) function, which 
#we could do or just take derivative by hand since h(x) doesn't change and should have a well defined, continous derivative 
#up to the boundary, which we should never cross anyways.

#I took the gradient myself, and came up with:
#dh/dx = 2.*(r_0 - x_1, r_1 - x_2), 
#where r_0 and r_1 are the x and y coords of the center of the circle. 

#The only problem is I thought we were using the reciprocal
#barrier function, and I don't see where the reciprocal is taken in the code, we declare h and h2 on line ~40 and without any fractions,
#then calling it on line ~176 for taking the gradient without inverting it. 

#Lastly, I couldn't find any documentation on Grad(), so maybe
#its a homemade function?

#class for creating an obstacle and calculating h(x), then taking the 
#gradient at a given point.

class obstacle:
    #initializes obj with default vals 0 for everything, 
    #can add values to constructor call as desired
    def __init__(object, R = 0, x = [0,0]):
        object.R = R
        object.x = x

    #h(x) if you need h(x) without the gradient for any reason
    def h_of_x(object,x):
        return object.R**2 - ((x[0] - object.x[0])**2 + (x[1] - object.x[1])**2)
    
    #called identically to Grad in MATLAB
    def Grad(object, coords):
      return [-2*(object.x[0] - coords[0]), -2*(object.x[1] - coords[1])]
    
#I basically made my own 2d array to overwrite whatever garbage python does, 
#so you should be able to use the eye method to do what you want,
class arr:
     def __init__(self):
        self.Arr = None
     #concatenate arrays, only works in one direction, call obj.Arr[i] 
     #to append horizontally
     def append(self, newArr):
        if self.Arr == None:
            self.Arr = newArr
        else:
            self.Arr = [self.Arr, newArr]
    #creates n-by-n identity matrix
     def eye(self,size):
         for i in range (0,size):
            newArr = [0]*size
            newArr[i] = 1
            self.append(newArr)

     def __getitem__(self, key):
        return self.Arr[key]
     
     def __add__(self, other):
         return self.append(other)

     
#2 norm of 2 points :)
def dist(x1, x2):
    return math.sqrt((x1[0] - x2[0])**2 + (x1[1] - x2[1])**2 )

#example using helper functions is provided below.

#obstacle, gradient and h(x) stuff:
circle = obstacle(1, [1,2])
print(circle.h_of_x([0.01,2]))

print(circle.Grad([0.01,2]))

#arr class using eye()
H = arr()
H.eye(3)
print("3x3 Identity Matrix: \n " + str(H.Arr))

#Two ways to random access: obj[x] and obj.Arr[x]
print( H[1])
print(H.Arr[1])

#first method of appending: obj1 + obj2
H2 = [1, 2, 3]
H = H + H2
print(H)

#arr class using append()
H = arr()
H.append([1,0,0])
H.append([0,1,0])
H.append([0,0,1])
print(H.Arr)


