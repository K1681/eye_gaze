import cv2
import mediapipe as mp
import pyautogui
import math
from time import perf_counter

cam = cv2.VideoCapture(0)
face_mesh = mp.solutions.face_mesh.FaceMesh(refine_landmarks=True)
callibration_x = callibration_y = 1
velocity_threshold = 2  # velocity_thereshold <= 100deg/s, horizontal_eye_motility = ±50deg ==> 100deg/s = 2units/s

def calculate_direction_v0(landmarks):
        global callibration_x, callibration_y

        left_eye_stationary_boundary_left = landmarks[243]
        left_eye_stationary_boundary_right = landmarks[130]
        left_eye_stationary_boundary_top = landmarks[223]
        left_eye_stationary_boundary_bottom = landmarks[230]

        left_eye_pupil_boundary_left = landmarks[471]
        left_eye_pupil_boundary_right = landmarks[469]
        left_eye_pupil_boundary_top = landmarks[470]
        left_eye_pupil_boundary_bottom = landmarks[472]

        left_eye_pupil_x = (left_eye_pupil_boundary_left.x + left_eye_pupil_boundary_right.x) / 2
        left_eye_pupil_y = (left_eye_pupil_boundary_top.y + left_eye_pupil_boundary_bottom.y) / 2
        left_eye_stationary_x = (left_eye_stationary_boundary_left.x + left_eye_stationary_boundary_right.x) / 2
        left_eye_stationary_y = (left_eye_stationary_boundary_top.y + left_eye_stationary_boundary_bottom.y) / 2
        left_eye_direction_x = ((left_eye_pupil_x - left_eye_stationary_x) * callibration_x) / (left_eye_stationary_boundary_left.x - left_eye_stationary_x)
        left_eye_direction_y = ((left_eye_pupil_y - left_eye_stationary_y) * callibration_y) / (left_eye_stationary_boundary_bottom.y - left_eye_stationary_y)

        return [left_eye_direction_x, left_eye_direction_y]

def get_direction(number_of_samples = 10, show_image = False):
    global cam, face_mesh, velocity_threshold

    number_of_tries = 100
    sample_counter = 0
    try_counter = 0
    previous_direction = [0, 0]
    previous_time = 0
    resultant_direction = [0, 0]

    while(try_counter < number_of_tries):
        _, frame = cam.read()
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        output = face_mesh.process(rgb_frame)
        landmark_points = output.multi_face_landmarks
        if landmark_points:
            previous_direction = calculate_direction_v0(landmark_points[0].landmark)
            previous_time = perf_counter()
            break
        try_counter += 1

    try_counter = 0
    while (sample_counter < number_of_samples) and (try_counter < number_of_tries):
        _, frame = cam.read()
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        output = face_mesh.process(rgb_frame)
        landmark_points = output.multi_face_landmarks
        if landmark_points:
            temp_direction = calculate_direction_v0(landmark_points[0].landmark)
            temp_time = perf_counter()
            if (math.sqrt(math.pow(temp_direction[0] - previous_direction[0], 2) + math.pow(temp_direction[1] - previous_direction[1], 2)) / (temp_time - previous_time)) <= velocity_threshold:
                resultant_direction[0] += temp_direction[0]
                resultant_direction[1] += temp_direction[1]
            sample_counter += 1
        if show_image:
            cv2.imshow('Eye Controlled Mouse', frame)
            cv2.waitKey(1)
        try_counter += 1

    if (try_counter == number_of_tries) & (sample_counter < number_of_samples): print(f"Collected {sample_counter} samples from {try_counter} tries. Samples/Tries = {sample_counter / try_counter}")
    return [resultant_direction[0] / number_of_samples, resultant_direction[1] / number_of_samples]

def callibrate():
    global callibration_x, callibration_y

    input("Press any key while looking as RIGHT as possible (vertically centered)")
    direction_x, _ = get_direction()
    callibration_x = 1 / direction_x
    print("Callibrated X")

    input("Press any key while looking as BOTTOM as possible (horizontally centered)")
    _, direction_y = get_direction()
    callibration_y = 1 / direction_y
    print("Callibrated Y")

def main():
    screen_w, screen_h = pyautogui.size()

    while True:
        # Get the average direction of the eyes
        direction = get_direction()
        print("Left Eye X:", direction[0], end="")
        print("\tLeft Eye Y:", direction[1])

        # Move the cursor based on the direction of the eyes
        screen_x = (screen_w / 2) + ((screen_w * direction[0]) / 2)
        screen_y = (screen_h / 2) + ((screen_h * direction[1]) / 2)

        # Bounding box. (Preventing PyAutoGUI fail-safe without setting pyautogui.FAILSAFE)
        if(screen_x < 10): screen_x = 10
        elif(screen_x > (screen_w - 10)): screen_x = screen_w - 10
        if(screen_y < 10): screen_y = 10
        elif(screen_y > (screen_h - 10)): screen_y = screen_h - 10

        pyautogui.moveTo(screen_x, screen_y)

if __name__ == "__main__":
    # callibrate()
    main()
