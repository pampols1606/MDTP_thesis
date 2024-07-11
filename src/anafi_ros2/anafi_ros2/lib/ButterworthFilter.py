import numpy as np

class ButterworthFilter():
    def __init__(self, order, a, b):
        self.n = order + 1
        self.x= np.zeros(self.n)
        self.y= np.zeros(self.n)
        self.a= np.array(a)
        self.b= np.array(b)

    def shift(self, x0, vector):
        vector[1:] = vector[:-1]
        vector[0] = x0
        return vector
    
    def filter(self, sample):
        self.x = self.shift(sample, self.x)
        self.y = self.shift(0.0, self.y)

        sum = self.b[0]*self.x[0]
        for i in range(1,self.n):
            sum += (self.b[i]*self.x[i]) - (self.a[i]*self.y[i])
        
        self.y[0] = sum
        return self.y[0]
