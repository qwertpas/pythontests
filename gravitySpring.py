
import math

M = 1
k = 1

segmentLength = 0.01
minStepAngle = -1
maxStepAngle = 2

x = 0
y = 1

for segment in range(1, 20):

    length = segment * segmentLength

    stepAngle = 0

    for guess in range(1, 20):
        x_guess = x + segmentLength * math.cos(stepAngle)
        y_guess = y + segmentLength * math.sin(stepAngle)

        # radius = math.hypot(x_guess, y_guess)
        # leverAngle = math.atan2(y_guess, x_guess)

        # desiredTorque = M * math.cos(leverAngle)
        # actualTorque = length * k * radius

        # errorTorque = desiredTorque - actualTorque

        errorTorque = M * x_guess - length * k * math.hypot(x_guess, y_guess)

        print("guess", guess, x_guess, y_guess, stepAngle, errorTorque)
        if(abs(errorTorque) < 1e-4): 
            break
    
        if(abs(errorTorque) > abs(lastErrorTorque)):
            stepAngle = stepAngle + maxStepAngle / (2**guess)
        else:
            stepAngle = stepAngle + maxStepAngle / (2**guess)


        stepAngle = stepAngle + errorTorque * 6

    x = x + segmentLength * math.cos(stepAngle)
    y = y + segmentLength * math.sin(stepAngle)

    errorTorque = M * x - length * k * math.hypot(x, y)

    print("segment", length, x, y, errorTorque)

