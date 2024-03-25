from rplidar import RPLidar
import numpy as np
import time
#import matplotlib.pyplot as plt
from rpi_hardware_pwm import HardwarePWM
import threading
#from gpiozero import LED, Button
from rpi_hardware_pwm import HardwarePWM
from stable_baselines3 import PPO, SAC
import spidev
#import smbus
#import keyboard

"""
bus = smbus.SMBus(1) 
def lecture_us_cm():
    try: 
        bus.write_byte_data(0x70, 0, 0x51)
        time.sleep(0.05)
        range1 = bus.read_byte_data(0x70, 2)
        range2 = bus.read_byte_data(0x70, 3)
        distance_cm = (range1 << 8) + range2
        if distance_cm < 1000:
            return distance_cm
        else:
            return 0
    except:
        return 0 """

steering_coeff = 1
acceleration_coeff = 1

# SPI init 
bus = 0
spi_len = 6 # Taille bits SPI
spi = spidev.SpiDev()
TxBuffer = [0]*spi_len
RxBuffer = [0]*spi_len
valeur_fourche = 0
valeur_arriere = 0

# Debuggage
SPI_print = 0

def convert(float_num): # Conversion au centieme de precision
    entier = int(float_num)
    decimale = int((float_num - int(float_num)) * 100)
    return entier, decimale

def recompose(): # Renvoie la fourche (mm/s) et la distance (cm) DANS CET ORDRE
    fo = RxBuffer[4]*100 + RxBuffer[5]
    return fo, TxBuffer[0] # ou 1 ou 2 ca marche aussi

def SPI_init():
    spi.open(bus, 1)
    spi.max_speed_hz = 1000000 # 1MHz
    spi.mode = 0

# Les cas de PWM locale et de mise en trame SPI ont été mis en évidence
# Trame définie : [ vitesse entier , vitesse décimale , 255 , 255 , angle entier , angle decimal ] 
# A comparer avec la reception STM

class Driver():
    
    def __init__(self):

        self.direction_prop = 1
        self.pwm_stop_prop = 7.47
        self.point_mort_prop = 0.34
        self.delta_pwm_max_prop = 1.4
        
        self.vitesse_max_m_s_hard = 8.3 #vitesse que peut atteindre la voiture
        self.vitesse_max_m_s_soft = 1 #vitesse maximale que l'on souhaite atteindre
        
        
        self.direction = 1
        self.angle_pwm_min = 6.4 #gauche
        self.angle_pwm_max = 9.2 #droite
        self.angle_degre_max = +18 #vers la gauche
        self.angle_pwm_centre= 7.8
        self.angle_degre=0
                
        #Configuration des PWM
        self.pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
        self.pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
        
        self.pwm_prop.start(self.pwm_stop_prop)
        
        
        self.vitesse_consigne = 0
        self.direction_consigne =0
        
        
        
    def demarrage_voiture(self):
        
        #Démarrage des PWM     
        self.pwm_prop.start(self.pwm_stop_prop)
        time.sleep(1.0)
        self.pwm_prop.start(10)
        time.sleep(0.1)
        self.pwm_prop.start(5)
        time.sleep(0.1)
        self.pwm_prop.start(self.pwm_stop_prop)
        self.pwm_dir.start(self.angle_pwm_centre)
        
        self.vitesse_consigne = 0
        self.direction_consigne =0
        
        print("PWM activé")
        
            
    def get_direction(self,obs=False):
        if obs:
            return self.direction_consigne/self.angle_degre_max
        else:
            return self.direction_consigne
    
    def get_vitesse(self,obs=False):
        if obs: 
            return self.vitesse_consigne/self.vitesse_max_m_s_hard
        else:
            return self.vitesse_consigne
        
    def set_direction_degre(self, angle_degre) :
        
        angle_pwm = self.angle_pwm_centre + self.direction * (self.angle_pwm_max - self.angle_pwm_min) * angle_degre /(2 * self.angle_degre_max )
        if angle_pwm > self.angle_pwm_max : 
            angle_pwm = self.angle_pwm_max
        if angle_pwm < self.angle_pwm_min :
            angle_pwm = self.angle_pwm_min

        # PWM Locale
        self.pwm_dir.change_duty_cycle(angle_pwm)

        # SPI Attrib
        TxBuffer[4], TxBuffer[5] = convert(angle_pwm)
    
    def set_vitesse_m_s(self,vitesse_m_s):
        if vitesse_m_s > self.vitesse_max_m_s_soft :
            vitesse_m_s = self.vitesse_max_m_s_soft
        elif vitesse_m_s < -self.vitesse_max_m_s_hard :
            vitesse_m_s = -self.vitesse_max_m_s_hard
        if vitesse_m_s == 0 :

            # PWM Locale
            self.pwm_prop.change_duty_cycle(self.pwm_stop_prop)

            # SPI Attrib
            TxBuffer[0], TxBuffer[1] = convert(self.pwm_stop_prop)

        elif vitesse_m_s > 0 :
            vitesse = vitesse_m_s * (self.delta_pwm_max_prop)/self.vitesse_max_m_s_hard

            # PWM Locale
            self.pwm_prop.change_duty_cycle(self.pwm_stop_prop + self.direction_prop*(self.point_mort_prop + vitesse ))

            # SPI Attrib
            TxBuffer[0], TxBuffer[1] = convert(self.pwm_stop_prop + self.direction_prop*(self.point_mort_prop + vitesse ))

        elif vitesse_m_s < 0 :
            vitesse = vitesse_m_s * (self.delta_pwm_max_prop)/self.vitesse_max_m_s_hard

            # PWM Locale
            self.pwm_prop.change_duty_cycle(self.pwm_stop_prop - self.direction_prop*(self.point_mort_prop - vitesse ))

            # SPI Attrib
            TxBuffer[0], TxBuffer[1] = convert(self.pwm_stop_prop - self.direction_prop*(self.point_mort_prop + vitesse ))

        self.vitesse_consigne = vitesse_m_s
            
        self.vitesse_consigne= vitesse_m_s
   
    def recule(self):
            """
            distance_arr=lecture_us_cm()
            print("dist avant boucle = ", distance_arr)
            debut = time.time()
            temps = time.time()
            while distance_arr<20 and temps-debut < 3:
                distance_arr=lecture_us_cm()
                print("dist dans boucle = ",distance_arr)
                temps = time.time()"""
            time.sleep(1)
            self.set_vitesse_m_s(-self.vitesse_max_m_s_hard)
            time.sleep(0.2)
            self.set_vitesse_m_s(0)
            time.sleep(0.1)
            self.set_vitesse_m_s(-1)
            self.vitesse_consigne=0
            
            
    def arret_voiture(self):
        self.set_direction_degre(0.0)
        self.pwm_prop.stop()
        self.pwm_dir.stop()
        print("PWM arrêtées")

class car_lidar():
    
    def __init__(self):

        self.lidar = RPLidar("/dev/ttyUSB0",baudrate=256000)
        self.tableau_lidar_mm = np.zeros(360)
        self.acqui_lidar = np.zeros(360)

        self.drapeau_nouveau_scan = False
        self.scan_avant_en_cours = False
        self.Run_lidar = False

    def start_lidar(self):
        self.lidar.connect()
        print(self.lidar.get_info())
        self.lidar.start_motor()
        time.sleep(2)

    def lidar_scan(self):
        while(self.Run_lidar == True):
            try:
                for _,_,angle_lidar,distance in self.lidar.iter_measures(scan_type='express'):
                    angle = min(359,max(0,359-int(angle_lidar)))
                    
                    if(angle >= 260) or (angle <= 100):
                        self.acqui_lidar[angle] = distance
                        
                        
                    if(angle<260) and (angle>110) and (self.scan_avant_en_cours == True):
                        self.drapeau_nouveau_scan = True
                        self.scan_avant_en_cours = False
                    if(angle >= 260) or (angle <= 100):
                        self.scan_avant_en_cours = True
                    if(self.Run_lidar == False):
                        break
            except:
                print("souci acquisition Lidar")
    
    def get_values(self):
            for i in range (-100,101):
                self.tableau_lidar_mm[i] = self.acqui_lidar[i]
            self.acqui_lidar = np.zeros(360)
            
            for i in range(-98,99):
                if self.tableau_lidar_mm[i] == 0:
                    if self.tableau_lidar_mm[i-1] != 0 and self.tableau_lidar_mm[i+1] != 0:
                        self.tableau_lidar_mm[i] = (self.tableau_lidar_mm[i]+self.tableau_lidar_mm[i])/2
            self.drapeau_nouveau_scan =  False
            return self.tableau_lidar_mm

    def get_drapeau(self):
        return self.drapeau_nouveau_scan
    
    def set_drapeau(self,valeur):
        self.drapeau_nouveau_scan = valeur
          
    def get_run(self):
        return self.Run_lidar
    
    def set_run(self, valeur):
        self.Run_lidar = valeur
        
    def arret_lidar(self):
        self.lidar.stop_motor()
        self.lidar.stop()
        time.sleep(1)
        self.lidar.disconnect()


def conduite_autonome() :
    global Run_Lidar
    global driver
    print("---------- Autonomous driving started ----------")
    tableau_lidar_mm = np.zeros(360)
    previous_lidar = np.zeros(201)
    while lidar.get_run() == True:
        if lidar.get_drapeau() == False:

            # Envoi et interpretation SPI
            RxBuffer = spi.xfer(TxBuffer)
            valeur_fourche, valeur_arriere = recompose()
            if SPI_print == 1: # DEBUG
                print(RxBuffer)
                print(valeur_fourche)
                print(valeur_arriere)

            time.sleep(0.01) # Je suppose que c'est la tempo ca c'est pour ca je fous le SPI ici 
        else:
            tableau_lidar_mm = lidar.get_values()
            for i in range(-100,101) :
                if tableau_lidar_mm[i] == 0 :
                    tableau_lidar_mm[i] = tableau_lidar_mm[i-1]
            #print(tableau_lidar_mm)
            min_secteur = [0]*10
                            
            for index_secteur in range(0,10) :
                angle_secteur = -90 + index_secteur*20
                min_secteur[index_secteur] = 8000
                for angle_lidar in range(angle_secteur-10,angle_secteur+10) :
                    if tableau_lidar_mm[angle_lidar] < min_secteur[index_secteur] and tableau_lidar_mm[angle_lidar] != 0 :
                        min_secteur[index_secteur] = tableau_lidar_mm[angle_lidar]
                if min_secteur[index_secteur] == 8000 : #aucune valeur correcte
                    min_secteur[index_secteur] = 0
            #print("min_secteur[] :")
            #print(min_secteur)
            
            if (min_secteur[4]<=160 and min_secteur[4] !=0)\
                    or (min_secteur[3]<=160 and min_secteur[3] !=0)\
                        or (min_secteur[2]<=160 and min_secteur[2] !=0) :
                angle_degre = -18
                driver.set_direction_degre(angle_degre)
                #   print("mur à droite")
                driver.recule()
                
            elif (min_secteur[5]<=160 and min_secteur[5] !=0)\
                    or (min_secteur[6]<=160 and min_secteur[6] !=0)\
                        or (min_secteur[7]<=160 and min_secteur[7] !=0) :
                angle_degre = +18
                driver.set_direction_degre(angle_degre)
                #  print("mur à gauche")
                driver.recule()
                
            else :
                
                current_lidar=(np.concatenate((tableau_lidar_mm[0:101],tableau_lidar_mm[260:360]),axis=None)).astype("float64")/8000 #8000
                previous_speed=np.array([float(driver.get_vitesse(obs=True))])
                previous_angle=np.array([float(driver.get_direction(obs=True))])
                
                obs={
                        "current_lidar":current_lidar,
                        "previous_lidar":previous_lidar,
                        "previous_speed":previous_speed,
                        "previous_angle":previous_angle,
                    }
                previous_lidar = current_lidar
                # inference
                action, _ = model.predict(obs,deterministic=True)

                # Appliquer l'action d'angle
                angle = float(driver.get_direction()) + action[1] * 18.0 * steering_coeff
                driver.set_direction_degre(-angle)
                

                # Appliquer l'action de vitesse
                nouvelle_vitesse = float(driver.get_vitesse()) + action[0] * acceleration_coeff

                if (nouvelle_vitesse <0.1) :
                    vitesse_m_s = 0.1
                else : 
                    vitesse_m_s = nouvelle_vitesse
                #print("vitesse  = ", vitesse_m_s)
                driver.set_vitesse_m_s(vitesse_m_s)

lidar = car_lidar()
driver = Driver()

print("Loading model..")
model = PPO.load("PPO4s")
#model = SAC.load("TrainSAC_1")
print("model loaded")

SPI_init()
print("SPI Loaded")

while True:
    try:
        print("appuyer sur i pour initialiser la voiture ou 'Ctrl+c' pour quitter le programme")
        key = input()
        while True:

            # initialisation de la voiture
            if key == "i":
                #connexion et démarrage du lidar
                lidar.start_lidar()
                lidar.set_run(True)
                print("lidar lancée")
                
                driver.demarrage_voiture()
                print("voiture lancée")

                thread_scan_lidar = threading.Thread(target=lidar.lidar_scan)
                thread_scan_lidar.start()
                time.sleep(1)
                
                print("thread lidar lancée")
                print("Appuyer sur touche 's' pour demarrer")
                key = input()
                if key == 's':
                    thread_conduite_autonome = threading.Thread(target = conduite_autonome)
                    #print("thread configurée")
                    thread_conduite_autonome.start()
                    print("thread conduite_autonome lancé")

                while True :
                    try :
                        print("Appuyer sur 'q' pour arrêter la voiture")
                        key = input()
                        #print("Appuyer sur Ctrl+C pour arreter le programme")
                        time.sleep(1)
                        if key =='q' :
                            lidar.set_run(False)
                            driver.set_vitesse_m_s(0)
                            driver.set_direction_degre(0.0)
                            break
                    except KeyboardInterrupt:
                        print("arrêt du programme")
                        lidar.set_run(False)
                        driver.set_vitesse_m_s(0)
                        break
                
           
                thread_scan_lidar.join()
                thread_conduite_autonome.join()
                lidar.arret_lidar()
                driver.arret_voiture()
                print("Voiture arrete")
                break
    except KeyboardInterrupt:
        break
print("\nProgram closed.")
