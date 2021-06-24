class GPS:
    def __init__(self, gyro, index, timeStep):
        self.sensor = gyro
        self.sensor.enable(timeStep)
        self.oldTime = 0.0
        self.index = index
        self.rotation = 0
        self.lastRads = 0
        self.x = 0
        self.z = 0

    # pass in the current wheels velocity.
    def update(self, velocity):
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = (self.sensor.getValues())[self.index] * timeElapsed
        self.lastRads = radsInTimestep
        finalRot = self.rotation + radsInTimestep
        self.rotation = normalizeRads(finalRot)
        self.oldTime = time
        self.x, self.z = getXY(velocity, radsToDegs(self.rotation))


# Getting components of vector
def getXY(magnitude, angle):
    x = cos(angle) * magnitude # get x component
    y = sin(angle) * magnitude # get y component
    return x, y