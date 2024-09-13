import cv2
import mediapipe as mp
import math
import time

# input variables ###################################################
angular_velocity_threshold_fixation_deg= 100  # <= 100 deg/sec
motility_deg = [[-50, 50], [-48, 42]]  # The range of motion of eye.
#####################################################################

# program variables
cam = cv2.VideoCapture(0)
face_mesh = mp.solutions.face_mesh.FaceMesh(refine_landmarks=True)

# precomputing for velocity
threshold_velocity = math.radians(angular_velocity_threshold_fixation_deg)
motility = [[0, 0], [0, 0]]  # moility = [[Tlx, Tux], [Tly, Tuy]].
for coorinate in range(2):
    for bound in range(2):
        motility[coorinate][bound] = math.radians(motility_deg[coorinate][bound])
v1 = (math.sin(motility[0][1]) - math.sin(motility[0][0])) / 2  # (sin(Tux) - sin(Tlx)) / 2
v2 = (math.sin(motility[0][1]) + math.sin(motility[0][0])) / 2  # (sin(Tux) + sin(Tlx)) / 2
v3 = (math.sin(motility[1][1]) - math.sin(motility[1][0])) / 2  # (sin(Tuy) - sin(Tly)) / 2
v4 = (math.sin(motility[1][1]) + math.sin(motility[1][0])) / 2  # (sin(Tuy) + sin(Tly)) / 2

# debugging
print("Debugg values:-")
print("V2 + V1", v2 + v1)
print("V2 - V1", v2 - v1)
print("V4 + V3", v4 + v3)
print("V4 - V3", v4 - v3)

# cash for callibration
callibration = [[[1, 1], [1, 1]], [[1, 1], [1, 1]]]
cc = list()

# Necessary functions.
def calculate_direction_uncallibrated(landmarks):
    xl = landmarks[468].x
    x1l = landmarks[130].x
    x2l = landmarks[243].x
    yl = landmarks[468].y
    y1l = landmarks[119].y
    y2l = landmarks[65].y

    xr = landmarks[473].x
    x1r = landmarks[463].x
    x2r = landmarks[359].x
    yr = landmarks[473].y
    y1r = landmarks[348].y
    y2r = landmarks[295].y

    return [[(2*xl - x1l - x2l)/(x2l - x1l), (2*yl - y1l - y2l)/(y2l - y1l)], [(2*xr - x1r - x2r)/(x2r - x1r), (2*yr - y1r - y2r)/(y2r - y1r)]]

def get_direction_uncallibrated(number_of_samples = 10):
    global cam, face_mesh
    sample_counter = 0
    resultant_direction = [[0, 0], [0, 0]]

    for _ in range(100):
        _, frame = cam.read()
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        output = face_mesh.process(rgb_frame)
        landmark_points = output.multi_face_landmarks
        if landmark_points:
            temp_direction = calculate_direction_uncallibrated(landmark_points[0].landmark)
            for eye in range(2):
                for coordinate in range(2):
                    resultant_direction[eye][coordinate] += temp_direction[eye][coordinate]
            sample_counter += 1
            if sample_counter == number_of_samples:
                break

    for eye in range(2):
        for coordinate in range(2):
            resultant_direction[eye][coordinate] /= number_of_samples

    return resultant_direction

def callibrate():
    global callibration, cc

    input("Press enter while looking LEFT")
    temp_direction = get_direction_uncallibrated()
    for eye in range(2):
        callibration[eye][0][0] = temp_direction[eye][0]

    input("Press enter while looking RIGHT")
    temp_direction = get_direction_uncallibrated()
    for eye in range(2):
        callibration[eye][0][1] = temp_direction[eye][0]

    input("Press enter while looking DOWN")
    temp_direction = get_direction_uncallibrated()
    for eye in range(2):
        callibration[eye][1][0] = temp_direction[eye][1]

    input("Press enter while looking UP")
    temp_direction = get_direction_uncallibrated()
    for eye in range(2):
        callibration[eye][1][1] = temp_direction[eye][1]

    a1l = callibration[0][0][0]
    a2l = callibration[0][0][1]
    a3l = callibration[0][1][0]
    a4l = callibration[0][1][1]

    a1r = callibration[1][0][0]
    a2r = callibration[1][0][1]
    a3r = callibration[1][1][0]
    a4r = callibration[1][1][1]

    cc.append(2 - a1l - a2l)
    cc.append(2 + a1l + a2l)
    cc.append(a1l - a2l)
    cc.append(a2l - a1l)
    cc.append(2 - a3l - a4l)
    cc.append(2 + a3l + a4l)
    cc.append(a3l - a4l)
    cc.append(a4l - a3l)

    cc.append(2 - a1r - a2r)
    cc.append(2 + a1r + a2r)
    cc.append(a1r - a2r)
    cc.append(a2r - a1r)
    cc.append(2 - a3r - a4r)
    cc.append(2 + a3r + a4r)
    cc.append(a3r - a4r)
    cc.append(a4r - a3r)

def calculate_direction(landmarks):
    global cc

    xl = landmarks[468].x
    x1l = landmarks[130].x
    x2l = landmarks[243].x
    yl = landmarks[468].y
    y1l = landmarks[119].y
    y2l = landmarks[65].y

    xr = landmarks[473].x
    x1r = landmarks[463].x
    x2r = landmarks[359].x
    yr = landmarks[473].y
    y1r = landmarks[348].y
    y2r = landmarks[295].y

    try:
        Uxl = (4*xl - x1l*cc[0] - x2l*cc[1]) / (x1l*cc[2] + x2l*cc[3])
        Uyl = (4*yl - y1l*cc[4] - y2l*cc[5]) / (y1l*cc[6] + y2l*cc[7])

        Uxr = (4*xr - x1r*cc[8] - x2r*cc[9]) / (x1r*cc[10] + x2r*cc[11])
        Uyr = (4*yr - y1r*cc[12] - y2r*cc[13]) / (y1r*cc[14] + y2r*cc[15])
    except ZeroDivisionError:
        print("Discarded a dirction calculation due to devide by zero error")
        return [[0, 0], [0, 0]]

    resultant_direction = [[Uxl, Uyl], [Uxr, Uyr]]
    print("resultant direction before range limiting: ", resultant_direction)
    # Safeguarding for furthur mathematical complications.
    for eye in range(2):
        for coordinate in range(2):
            if resultant_direction[eye][coordinate] > 1:
                resultant_direction[eye][coordinate] = 1
            elif resultant_direction[eye][coordinate] < -1:
                resultant_direction[eye][coordinate] = -1
    print("resultant direction after range limiting: ", resultant_direction)
    return resultant_direction

# Todo: domain error in math.asin() even though i/p doesnt exceed [-1, 1].
def calculate_angular_velocity(initial_direction, final_direction, inital_time, final_time):
    global v1, v2, v3, v4
    # Tx(Ux) = sin^-1(((sin(Tux) - sin(Tlx))/2) * Ux + ((sin(Tux) + sin(Tlx))/2))
    # Ty(Uy) = sin^-1(((sin(Tuy) - sin(Tly))/2) * Uy + ((sin(Tuy) + sin(Tly))/2))

    # left eye:
    dTxl = math.asin(v1 * final_direction[0][0] + v2) - math.asin(v1 * initial_direction[0][0] + v2)
    dTyl = math.asin(v3 * final_direction[0][1] + v4) - math.asin(v3 * initial_direction[0][1] + v4)
    Vl = math.sqrt((dTxl ** 2) + (dTyl ** 2)) / (final_time - inital_time)

    # right eye
    dTxr = math.asin(v1 * final_direction[1][0] + v2) - math.asin(v1 * initial_direction[1][0] + v2)
    dTyr = math.asin(v3 * final_direction[1][1] + v4) - math.asin(v3 * initial_direction[1][1] + v4)
    Vr = math.sqrt((dTxr ** 2) + (dTyr ** 2)) / (final_time - inital_time)

    return (Vl + Vr) / 2

def get_direction(number_of_samples = 10):
    global cam, face_mesh, threshold_velocity
    sample_counter = valid_sample_counter = 0
    resultant_direction = [[0, 0], [0, 0]]

    # Getting initial direction.
    for _ in range(100):
        _, frame = cam.read()
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        output = face_mesh.process(rgb_frame)
        landmark_points = output.multi_face_landmarks
        if landmark_points:
            previous_direction = calculate_direction(landmark_points[0].landmark)
            previous_time = time.perf_counter()
            break

    # Getting following directions with velocity filtering.
    for _ in range(100):
        _, frame = cam.read()
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        output = face_mesh.process(rgb_frame)
        landmark_points = output.multi_face_landmarks

        if landmark_points:
            temp_direction = calculate_direction(landmark_points[0].landmark)
            temp_time = time.perf_counter()

            if(calculate_angular_velocity(previous_direction, temp_direction, previous_time, temp_time) <= threshold_velocity):
                for eye in range(2):
                    for coordinate in range(2):
                        resultant_direction[eye][coordinate] += temp_direction[eye][coordinate]
                valid_sample_counter += 1

            previous_direction = temp_direction
            previous_time = temp_time
            sample_counter += 1

            if sample_counter == number_of_samples:
                break

    for eye in range(2):
        for coordinate in range(2):
            resultant_direction[eye][coordinate] /= valid_sample_counter

    return resultant_direction
