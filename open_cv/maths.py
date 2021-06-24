DistanceCamera1 = 44
DistanceCamera2 = 45
DistanceCamera3 = 40


def Error_Rate(DistanceCamera1,DistanceCamera2,DistanceCamera3):
    X1 = DistanceCamera1
    X2 = DistanceCamera2
    X3 = DistanceCamera3
    Xp = (X1 + X2 + X3)/3

    E1 = Xp - X1
    E2 = Xp - X2
    E3 = Xp - X3
    A = (E1+E2+E3)
    E = abs(A)/3
    #Percentage_error = (abs(E)/Xp)*100
    finalNumberSum = Xp + E
    finalNumberRest = Xp - E
    numbers = [finalNumberSum, finalNumberRest]
    return numbers

def Final_Error_Filter(Error_Rate):
    lower_camera = [DistanceCamera1, DistanceCamera2, DistanceCamera3]
    lower_camera = min(lower_camera)
    Porcentaje_Low = (Error_Rate[1]*lower_camera)/100
    Final_Number = lower_camera + Porcentaje_Low
    return Final_Number

print(Error_Rate(DistanceCamera1,DistanceCamera2,DistanceCamera3))
print(Final_Error_Filter(Error_Rate(DistanceCamera1,DistanceCamera2,DistanceCamera3)))
