import cv2
import numpy as np
import face_recognition
import os
import serial
from datetime import datetime

# Motion detection parameters
motion_threshold = 200  # Adjust this threshold based on your environment

# Face recognition parameters
path = "ImagesAttendance"
images = []
classNames = []
myList = os.listdir(path)
print(myList)
for cl in myList:
    curImg = cv2.imread(f'{path}/{cl}')
    images.append(curImg)
    classNames.append(os.path.splitext(cl)[0])
classNames.append('Unknown')  # Add 'Unknown' class
print(classNames)

def findEncodings(images):
    encodeList = []
    for img in images:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        encode = face_recognition.face_encodings(img)[0]
        encodeList.append(encode)
    return encodeList

def markAttendance(name):
    with open('attendance.csv', 'r+') as f:
        myDataList = f.readlines()
        nameList = []
        for line in myDataList:
            entry = line.split(',')
            nameList.append(entry[0])
        if name not in nameList:
            now = datetime.now()
            dateString = now.strftime('%H:%M:%S')
            f.writelines(f'\n{name},{dateString}')

# Establish serial connection with FTDI
serial_port = "/dev/ttyACM0"  # Replace with the appropriate COM port
baud_rate = 9600
ser = serial.Serial(serial_port, baud_rate)

encodeListKnown = findEncodings(images)
print('Encoding Complete')

# Mapping names to specific serial data
name_to_serial = {
    'IAN': b'1',
    'JAMES': b'2',
    'LUCY': b'3',
    'JOHN': b'4',
    'JANET': b'5'
}

# Initialize variables for motion detection and face detection
previous_frame = None
face_detected = False
recognized_faces = {}  # Dictionary to track recognition state

cap = cv2.VideoCapture(0)

while True:
    success, img = cap.read()
    imgS = cv2.resize(img, (0, 0), None, 0.25, 0.25)
    imgS = cv2.cvtColor(imgS, cv2.COLOR_BGR2RGB)

    # Convert the frame to grayscale for motion detection
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    # Perform motion detection
    if previous_frame is None:
        previous_frame = gray
        continue

    frame_delta = cv2.absdiff(previous_frame, gray)
    thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    motion_detected = False
    for contour in contours:
        if cv2.contourArea(contour) > motion_threshold:
            motion_detected = True
            break

    if motion_detected:
        # Motion detected, perform face recognition
        facesCurFrame = face_recognition.face_locations(imgS)
        encodesCurFrame = face_recognition.face_encodings(imgS, facesCurFrame)

        face_detected = False
        for encodeFace, faceloc in zip(encodesCurFrame, facesCurFrame):
            matches = face_recognition.compare_faces(encodeListKnown, encodeFace)
            faceDis = face_recognition.face_distance(encodeListKnown, encodeFace)
            matchIndex = np.argmin(faceDis)

            if matches[matchIndex]:
                # Face recognized
                name = classNames[matchIndex].upper()
                if name in recognized_faces and recognized_faces[name] >= 3:
                    continue  # Skip if already printed/sent 3 times

                if name not in recognized_faces:
                    recognized_faces[name] = 1
                else:
                    recognized_faces[name] += 1

                serial_data = name_to_serial.get(name, b'6')  # Default to '6' if name not found
                for _ in range(3):
                    ser.write(serial_data)  # Send corresponding serial data multiple times
                    print(f"Match found: {name}, sending serial data: {serial_data.decode()}")  # Print serial data

                face_detected = True

            else:
                # Face not recognized
                name = 'Unknown'
                ser.write(b'7')  # Send '7' for unknown face
                print("No match found, sending serial data: 7")

            y1, x2, y2, x1 = faceloc
            y1, x2, y2, x1 = y1 * 4, x2 * 4, y2 * 4, x1 * 4
            if name == 'Unknown':
                # Draw red square for unrecognized face
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.rectangle(img, (x1, y2-35), (x2, y2), (0, 0, 255), cv2.FILLED)
                cv2.putText(img, name, (x1+6, y2-6), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)
            else:
                # Draw green square for recognized face
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.rectangle(img, (x1, y2-35), (x2, y2), (0, 255, 0), cv2.FILLED)
                cv2.putText(img, name, (x1+6, y2-6), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)
                markAttendance(name)

    else:
        # No motion detected, reset recognition state
        recognized_faces = {}

    cv2.imshow('Webcam', img)
    if cv2.waitKey(1) == ord('q'):
        break

    previous_frame = gray

cap.release()
cv2.destroyAllWindows()
