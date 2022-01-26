#параметры маркера
class MarkerPose:
    def __init__(self, x, y, theta, quality, order = None):
        #координаты метки
        self.x = x
        self.y = y
        self.theta = theta
        self.quality = quality #качество распознавания
        self.order = order #номер метки, которая задана

    def scale_position(self, scale_factor):
        self.x = self.x * scale_factor
        self.y = self.y * scale_factor


