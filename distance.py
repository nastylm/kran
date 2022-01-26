

import cv2

# дистанция от камеры до объекта измерение
KNOWN_DISTANCE = 35  # см
# ширина(плоскость объекта)
KNOWN_WIDTH = 17  # см
# Цвета
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
fonts = cv2.FONT_HERSHEY_COMPLEX
cap = cv2.VideoCapture(0)

# детектор
detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")


# фокусное расстояние
def focal_length(measured_distance, real_width, width_in_rf_image):

    focal_length_value = (width_in_rf_image * measured_distance) / real_width
    return focal_length_value


# функция оценки расстояния
def distance_finder(focal_length, real_width, width_in_frame):
    """
    This Function simply Estimates the distance between object and camera using arguments(focal_length, Actual_object_width, Object_width_in_the_image)
    :param1 focal_length(float): return by the focal_length_Finder function

    :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 5.7 Inches)
    :param3 object_Width_Frame(int): width of object in the image(frame in our case, using Video feed)
    :return Distance(float) : distance Estimated
    """
    distance = (real_width * focal_length) / width_in_frame
    return distance


# функция детектора
def obj_data(image):
    """
    This function Detect the face
    :param Takes image as argument.
    :returns face_width in the pixels
    """

    obj_width = 0
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    objects = detector.detectMultiScale(gray_image, 1.3, 5)
    for (x, y, h, w) in objects:
        cv2.rectangle(image, (x, y), (x + w, y + h), WHITE, 1)
        obj_width = w

    return obj_width


# чтение референса
ref_image = cv2.imread("???????")

ref_image_width = obj_data(ref_image)
focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_width)
print(focal_length_found)
cv2.imshow("ref_image", ref_image)

while True:
    _, frame = cap.read()

    # вызов функции детектора
    width_in_frame = obj_data(frame)
    # нахождение расстояния
    if width_in_frame != 0:
        Distance = distance_finder(focal_length_found, KNOWN_WIDTH, width_in_frame)
        # текст на экране
        cv2.putText(
            frame, f"Distance = {round(Distance,2)} CM", (50, 50), fonts, 1, (WHITE), 2
        )
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
