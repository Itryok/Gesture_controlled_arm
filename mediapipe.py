import cv2
import mediapipe as mp
import math
import serial
import time

mp_hands=mp.solutions.hands
mpDraw = mp.solutions.drawing_utils
# Configure the serial port
arduino_port = '/dev/ttyUSB0'  # Change this to your Arduino's port
baud_rate = 9600
ser = serial.Serial(arduino_port, baud_rate, timeout=1)

# mp_hands=mp.solutions.hands
# mpDraw = mp.solutions.drawing_utils
# sc=0

def cond_rock(ip,it,mp,mt,rp,rt,pp,pt):
    if(ip<it) and (mp<mt) and (rp<rt) and (pp <pt):
        return 45
    return 0
def cond_paper(ip,it,mp,mt,rp,rt,pp,pt):
    if(ip>it) and (mp>mt) and (rp>rt) and (pp>pt):
        return True
    return False
def cond_scissor(ip,it,mp,mt,rp,rt,pp,pt):
    if(ip>it) and (mp>mt) and (rp<rt) and (pp <pt):
        return True
    return False

# Function to calculate angle between three points
def calculate_angle(p1, p2, p3):
    v1 = [p2[0] - p1[0], p2[1] - p1[1]]
    v2 = [p3[0] - p2[0], p3[1] - p2[1]]
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    angle = math.atan2(cross_product, dot_product)
    return math.degrees(angle)

# Initialize MediaPipe Holistic model
mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils
holistic = mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Initialize video capture
cap = cv2.VideoCapture(0)
while True:
    i=0
    # Read a frame from video capture
    ret, frame = cap.read()

    # Convert frame to RGB
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process frame using Holistic model
    results = holistic.process(image)
    # Extract landmark coordinates
    if results.pose_landmarks:
        pose_landmarks = results.pose_landmarks.landmark
        shoulder2 = [int(pose_landmarks[mp_holistic.PoseLandmark.RIGHT_SHOULDER].x * frame.shape[1]),
                     int(pose_landmarks[mp_holistic.PoseLandmark.RIGHT_SHOULDER].y * frame.shape[0])]
        elbow2 = [int(pose_landmarks[mp_holistic.PoseLandmark.RIGHT_ELBOW].x * frame.shape[1]),
                  int(pose_landmarks[mp_holistic.PoseLandmark.RIGHT_ELBOW].y * frame.shape[0])]
        wrist2 = [int(pose_landmarks[mp_holistic.PoseLandmark.RIGHT_WRIST].x * frame.shape[1]),
                  int(pose_landmarks[mp_holistic.PoseLandmark.RIGHT_WRIST].y * frame.shape[0])]
        hip2 = [int(pose_landmarks[mp_holistic.PoseLandmark.RIGHT_HIP].x * frame.shape[1]),
               int(pose_landmarks[mp_holistic.PoseLandmark.RIGHT_HIP].y * frame.shape[0])]
        index_finger_tip = [int(pose_landmarks[20].x * frame.shape[1]),
                    int(pose_landmarks[20].y * frame.shape[0])]
        wrist_flexion_angle = calculate_angle(elbow2, wrist2, index_finger_tip)  # Adjust based on chosen landmarks
        
        txt=""
        sc=0
        with mp_hands.Hands(static_image_mode=True,max_num_hands=1,min_detection_confidence=0.5)as hands:   
            results1= hands.process(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
            if results1.multi_hand_landmarks:
                hand_landmarks=results1.multi_hand_landmarks[0]        
                ip = [int(hand_landmarks.landmark[6].x*frame.shape[1]), int(hand_landmarks.landmark[6].y*frame.shape[0])]
                it = [int(hand_landmarks.landmark[8].x*frame.shape[1]), int(hand_landmarks.landmark[8].y*frame.shape[0])]
                MP = [int(hand_landmarks.landmark[10].x*frame.shape[1]), int(hand_landmarks.landmark[10].y*frame.shape[0])]
                mt = [int(hand_landmarks.landmark[12].x*frame.shape[1]), int(hand_landmarks.landmark[11].y*frame.shape[0])]
                rp = [int(hand_landmarks.landmark[14].x*frame.shape[1]), int(hand_landmarks.landmark[14].y*frame.shape[0])]
                rt = [int(hand_landmarks.landmark[16].x*frame.shape[1]), int(hand_landmarks.landmark[16].y*frame.shape[0])]
                pp = [int(hand_landmarks.landmark[18].x*frame.shape[1]), int(hand_landmarks.landmark[18].y*frame.shape[0])]
                pt = [int(hand_landmarks.landmark[20].x*frame.shape[1]), int(hand_landmarks.landmark[20].y*frame.shape[0])]
                txt = ""
                hand=[ip,it,MP,mt,rp,rt,pp,pt]
                i=cond_rock(ip[1],it[1],MP[1],mt[1],rp[1],rt[1],pp[1],pt[1])
                for finger in hand:
                    cv2.circle(frame, tuple(finger), 5, (0, 0, 255), -1)
        # Draw and display (update text to reflect wrist flexion)
        angle2 = calculate_angle(shoulder2, elbow2, wrist2)
        angle_hip_shoulder2 = calculate_angle(hip2, shoulder2, elbow2)
        angles = [ 180 - angle2,  angle_hip_shoulder2, i, 0]
        for angle in angles:
            ser.write(str(angle).encode() + b',')
        ser.write(b'\n')  # Add a newline character to indicate the end of angles
        
        
        # Draw the tracked joints as nodes and the angles
        cv2.circle(frame, tuple(shoulder2), 5, (0, 0, 255), -1)
        cv2.circle(frame, tuple(elbow2), 5, (0, 0, 255), -1)
        cv2.circle(frame, tuple(wrist2), 5, (0, 0, 255), -1)
        cv2.circle(frame, tuple(hip2), 5, (255, 0, 0), -1)
        cv2.circle(frame, tuple(index_finger_tip), 5, (0, 0, 255), -1)
        cv2.line(frame, tuple(shoulder2), tuple(elbow2), (0, 0, 255), 2)
        cv2.line(frame, tuple(elbow2), tuple(wrist2), (0, 0, 255), 2)
        cv2.line(frame, tuple(hip2), tuple(shoulder2), (0, 0, 255), 2)
        cv2.line(frame, tuple(wrist2), tuple(index_finger_tip), (0, 0, 255), 2)

        cv2.putText(frame, f"Angle 2: {angle2:.2f} degrees", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, f"Angle hip-shoulder2: {angle_hip_shoulder2:.2f} degrees", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(frame, f"Wrist Flexion: {wrist_flexion_angle:.2f} degrees", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, f"Wrist Flexion: {sc:.2f} degrees", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        

    # Display the resulting frame
    cv2.imshow('Arm Tracking', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close all windows
cap.release()
cv2.destroyAllWindows()