import numpy as np

class Point:
	def __init__(self, x, y, z=0):
		self.x = x
		self.y = y
		self.z = z
		self.vec = np.array([x, y, z])
    
	def getX(self):
		return self.x
    
	def getY(self):
		return self.y
    
	def getZ(self):
		return self.z
    
	def setPose(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z
		
	def getVecMagnitude(self, planar=False):
		if(planar):
			return math.sqrt(self.x**2 + self.y**2)
		else:
			return math.sqrt(self.x**2 + self.y**2 + self.z**2)
