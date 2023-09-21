import easygopigo3
import easysensors
import time
from math import sqrt
from math import pi



class Robot:
    def __init__(self):
        self.gopigo = easygopigo3.EasyGoPiGo3()
        self.lidar = self.gopigo.init_distance_sensor()
        self.servo = self.gopigo.init_servo()
        self.led = self.gopigo.init_led()

    def clignotant(self):
        self.gopigo.open_eyes()
        time.sleep(0.5)
        self.gopigo.close_eyes()
        time.sleep(0.5)

    def detection(self):
        if self.lidar.read() <= 15:
            self.gopigo.stop()
            return True
        else:
            return False

    def auto_pilote(self):
        self.detection()
        time.sleep(0.75)
        if self.detection() is True:
            time.sleep(0.75)
            self.servo.rotate_servo(5)
            time.sleep(0.75)
            if self.detection() is False:
                time.sleep(0.75)
                self.gopigo.right()
                time.sleep(0.75)
            else:
                time.sleep(0.75)
                self.servo.rotate_servo(175)
                time.sleep(0.75)
                if self.detection() is False:
                    time.sleep(0.75)
                    self.gopigo.left()
                    time.sleep(0.75)
                else:
                    time.sleep(0.75)
                    self.gopigo.turn_degrees(180)
                    time.sleep(0.75)

    def move(self):
        self.servo.rotate_servo(90)
        self.gopigo.set_speed(300)
        self.gopigo.forward()
        if self.detection() is True:
            self.gopigo.stop()
            self.auto_pilote()

    def search_place(self):
        self.servo.rotate_servo(1)
        time.sleep(0.30) 
        self.gopigo.set_speed(150)
        time.sleep(0.30)
        self.gopigo.forward() 
        time.sleep(0.30) 
        k = self.lidar.read()
        while True:
            if self.lidar.read() >= 30:
                self.gopigo.stop()
                time.sleep(0.30)
                self.gopigo.drive_cm(self.lidar.read()/4)
                break
        return k

    def fct_yx(self, x, a, b): 
        return sqrt((1 - (x/a)**2) * b**2)
 
    def fct_der1_yx(self, x, a, b):
        return (-x * b**2) / (a**2 * self.fct_yx(x, a, b))
 
    def fct_der2_yz(self, x, a, b):
        return ((x**2 * b**4 / self.fct_yx(x, a, b)) + (b*a)**2 * self.fct_yx(x, a, b)) / (a**2 * self.fct_yx(x, a, b))**2

    def rayon_courbure(self, x, a, b):
        return abs((1 + self.fct_der1_yx(x, a, b)**2)**(3/2) / self.fct_der2_yz(x, a, b))

    def rangement_bataille(self):
        a = self.search_place()
        b = 8
        h = (a - b)**2 / (a + b)**2
        périmètre_quart_ellipse = pi / 4 * (a + b) * (1 + 3 * h / (10 + sqrt(4 - 3 * h ** 2)))
        x = [i for i in range(-a, 1, 2)]                             # On effectue un arc de cercle tous les 2cm
        xr = x[1:]
        xr.insert(0, -(a - 0.1))                                     # On ajoute le premier arc de cercle manuellement
        yr = [self.rayon_courbure(i, a, b) for i in xr]              # Création de la liste avec les rayons de courbures
        dl = périmètre_quart_ellipse / len(yr)
        dtheta = [(dl / i) * (180 / pi) for i in yr]                 # Création de la liste avec les petits angles
        time.sleep(0.75)

        for i in range(len(yr)):                                     # On boucle sur toute la longeur de la liste
            time.sleep(0.20)
            self.gopigo.orbit(dtheta[i], yr[i])

        # Correction
        self.gopigo.set_speed(100)
        self.gopigo.turn_degrees(90 - sum(dtheta))                   # Pour finir la rotation à 90°
        time.sleep(0.20)
        self.servo.rotate_servo(90)
        time.sleep(0.20)
        obs = self.lidar.read()
        self.gopigo.drive_cm(obs-3)                          # Pour que le robot s'arrete à 3cm avant la fin de la place


if __name__ == "__main__":
    t0 = time.time()
    x = Robot()
    #x.search_place()
    x.rangement_bataille()
    """while True:
        t1 = time.time()
        x.move()
        if t1 - t0 >= 20:
            x.gopigo.stop()
            break"""
