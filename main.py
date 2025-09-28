import serial
import time
import cv2
import mediapipe as mp

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.last_error_x = 0
        self.last_error_y = 0
        self.integral_x = 0
        self.integral_y = 0

    def update(self, measurement):
        error_x = self.setpoint[0] - measurement[0]
        error_y = self.setpoint[1] - measurement[1]
        # self.integral_x += error_x
        # self.integral_y += error_y
        # derivative_x = error_x - self.last_error_x
        # derivative_y = error_y - self.last_error_y
        # output_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        # output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
        output_x = self.kp * error_x
        output_y = self.kp * error_y

        self.last_error_x = error_x
        self.last_error_y = error_y

        return (output_x, output_y)
    
def connect_to_serial():
    try:
        ser = serial.Serial("COM9", 115200, timeout=1)
        if ser.is_open:
            print("Connected to serial port successfully.")

            while True:
                # user_input = input("Please input motor index and target speed, separated by a space (or type 'exit' to quit): ")
                # if user_input.lower() == "exit":
                #     break

                # line = ser.readline().decode('utf-8').rstrip()
                # if line:
                #     print(line)

                try:
                    # value = int(user_input)
                    # value = 10000
                    # data = struct.pack('>i', value)  
                    data = f"{0} {0}\n".encode('utf-8')
                    ser.write(data)
  
                except ValueError:
                    print("Invalid input. Please enter an integer.")
                    
    except serial.SerialException as e:
        print(f"Error connecting to serial port: {e}")
    
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

def show_camera_with_face_detection():
    cap = cv2.VideoCapture(1)  

    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    writeDelayCounter = 0

    with mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.8) as face_detection:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = face_detection.process(frame_rgb)

            if results.detections:
                # Find the face with the highest confidence
                best_detection = max(results.detections, key=lambda d: d.score[0])
                h, w, _ = frame.shape
                bbox = best_detection.location_data.relative_bounding_box
                x = int(bbox.xmin * w)
                y = int(bbox.ymin * h)
                box_width = int(bbox.width * w)
                box_height = int(bbox.height * h)
                print(f"Detected face at Center:({x + box_width//2}, {y + box_height//2})")

                measurement = (x + box_width//2, y + box_height//2)
                control = pid.update(measurement)
                print(f"PID control output: {control}")

                writeDelayCounter += 1

                # Control x-axis motor
                data_x = f"{1} {int(10 * control[0])}\n".encode('utf-8')
                # control[0] > 0 means looking left
                # control[0] == 0 means staying still
                # control[0] < 0 means looking right
                if writeDelayCounter == 40:
                    ser.write(data_x)

                # Control y-axis motor
                data_y = f"{0} {int(-10 * control[1])}\n".encode('utf-8')
                # control[1] < 0 means looking up
                # control[1] == 0 means staying still
                # control[1] > 0 means bowing head
                if writeDelayCounter == 20:
                    ser.write(data_y)

                if writeDelayCounter >= 40:
                    writeDelayCounter = 0
                mp_drawing.draw_detection(frame, best_detection)
            else:
                data_x = f"{1} {0}\n".encode('utf-8')
                data_y = f"{0} {0}\n".encode('utf-8')
                ser.write(data_x)
                time.sleep(0.5)
                ser.write(data_y)
                print("No face detected")

            cv2.imshow('MediaPipe Face Detection', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()



if __name__ == "__main__":

    ser = serial.Serial("COM9", 115200, timeout=1)
    if ser.is_open:
        print("Connected to serial port successfully.")

    # Assuming a 640x480 frame, center is (320, 240)
    pid = PID(kp=1.0, ki=0.1, kd=0.05, setpoint=(320, 240)) 

    show_camera_with_face_detection()

    ser.close()

    # # Test Code
    # while True:
    #     data = f"{0} {-500}\n".encode('utf-8')
    #     ser.write(data)
    #     time.sleep(1)

    #     data = f"{1} {-500}\n".encode('utf-8')
    #     ser.write(data)

    #     time.sleep(5)

    #     data = f"{0} {500}\n".encode('utf-8')
    #     ser.write(data)
    #     time.sleep(1)

    #     data = f"{1} {500}\n".encode('utf-8')
    #     ser.write(data)

    #     time.sleep(5)
 
