from KinematicModel import KinematicModel


kModel = KinematicModel()

position = (-200, 300, 1200)
orientation = (0, 0, 0, 1)

theta = kModel.inverse_kinematics_ABB_1400(position, orientation)
print(theta)