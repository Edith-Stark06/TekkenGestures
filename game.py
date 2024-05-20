import cv2
import mediapipe as mp
import numpy as np
import pyautogui
import threading

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

gesture_detected = False
gesture_name = ""

def recognize_gesture(results):
    global gesture_detected, gesture_name
    
    if results.pose_landmarks is not None:
        left_hand_landmarks = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].y
        right_hand_landmarks = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y
        left_shoulder_y = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y
        right_shoulder_y = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y
        
        if left_hand_landmarks < left_shoulder_y and right_hand_landmarks < right_shoulder_y:
            gesture_detected = True
            gesture_name = "Punch"
        elif left_hand_landmarks < left_shoulder_y and right_hand_landmarks > right_shoulder_y:
            gesture_detected = True
            gesture_name = "Kick"
        elif left_hand_landmarks > left_shoulder_y and right_hand_landmarks < right_shoulder_y:
            gesture_detected = True
            gesture_name = "Power"
        elif left_hand_landmarks < left_shoulder_y and right_hand_landmarks < right_shoulder_y:
            gesture_detected = True
            gesture_name = "Move Forward"
        elif left_hand_landmarks > left_shoulder_y and right_hand_landmarks > right_shoulder_y:
            gesture_detected = True
            gesture_name = "Move Backward"
        
        else:
            gesture_detected = False
            gesture_name = ""

def gesture_recognition(cap):
    with mp_pose.Pose(min_detection_confidence=0.3, min_tracking_confidence=0.3) as pose:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame.")
                break
            
            # Resize frame to a smaller resolution
            frame = cv2.resize(frame, (640, 480))
            
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False

            results = pose.process(image)
            
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            recognize_gesture(results)

            if gesture_detected:
                if gesture_name == "Punch":
                    pyautogui.press('i')
                elif gesture_name == "Move Forward":
                    pyautogui.press('a')
                elif gesture_name == "Kick":
                    pyautogui.press('j')
                elif gesture_name == "Power":
                    pyautogui.press('o')
                elif gesture_name == "Move Backward":
                    pyautogui.press('d')

            cv2.putText(image, gesture_name, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow("Gesture Recognition", image)

            if cv2.waitKey(1) == ord("q"):
                break

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    gesture_thread = threading.Thread(target=gesture_recognition, args=(cap,))
    gesture_thread.start()

    gesture_thread.join()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
