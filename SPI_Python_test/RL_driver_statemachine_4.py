from rplidar import RPLidar
import time
import numpy as np
from rpi_hardware_pwm import HardwarePWM
from stable_baselines3 import PPO
import sys
import select
import spidev
import threading # Necessaire pour le SPI qui a besoin de la tempo
                 # On aura forcement un temps de reaction plus élevé ce qui est défavorisant a cette approche ...

distance_max_lidar_mm = 12000
steering_coeff = 1
speed_coeff = 1

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

def SPI_init():
    spi.open(bus, 1)
    spi.max_speed_hz = 1000000 # 1MHz
    spi.mode = 0

def recompose(): # Renvoie la fourche (mm/s) et la distance (cm) DANS CET ORDRE
    fo = RxBuffer[4]*100 + RxBuffer[5]
    return fo, TxBuffer[0] # ou 1 ou 2 ca marche aussi

# Obsolete (marche meme pas de toute facon)
def SPI_thread():
    global spi
    global TxBuffer
    global RxBuffer
    global valeur_fourche
    global valeur_arriere
    SPI_init()
    while True:
        RxBuffer = spi.xfer(TxBuffer)
        if SPI_print == 1:
            print(RxBuffer)
        time.sleep(0.1) # Bottleneck ici forcement

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
        # Démarrage des PWM (un peu brutal 10 et 5 non?)
        self.pwm_prop.start(self.pwm_stop_prop)
        time.sleep(1.0)
        self.pwm_prop.start(6)
        time.sleep(0.1)
        self.pwm_prop.start(3)
        time.sleep(0.1)
        self.pwm_prop.start(self.pwm_stop_prop)
        self.pwm_dir.start(self.angle_pwm_centre)
        
        self.vitesse_consigne = 0
        self.direction_consigne = 0
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
    
    def get_vitesse_max(self):
        return self.vitesse_max_m_s_hard
        
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
   
            
    def arret_voiture(self):
        self.set_direction_degre(0.0)
        self.pwm_prop.stop()
        self.pwm_dir.stop()
        print("PWM arrêtées")


		# Get Lidar observation
def get_observation(prev_lid, init=False):
    global tableau_lidar_mm

    for i in range(-98, 99):
        if tableau_lidar_mm[i] == 0:
            if tableau_lidar_mm[i-1] != 0 and tableau_lidar_mm[i+1] != 0:
                tableau_lidar_mm[i] = (tableau_lidar_mm[i-1] + tableau_lidar_mm[i+1])/2
    
    for i in range(-100, 101):
        if tableau_lidar_mm[i] == 0:
            tableau_lidar_mm[i] = tableau_lidar_mm[i-1]

    if init:
        current_lidar=(np.concatenate((tableau_lidar_mm[0:101],tableau_lidar_mm[260:360]),axis=None)).astype("float64")/distance_max_lidar_mm
        previous_lidar=(np.concatenate((tableau_lidar_mm[0:101],tableau_lidar_mm[260:360]),axis=None)).astype("float64")/distance_max_lidar_mm
        previous_speed=np.zeros(1)
        previous_angle=np.zeros(1)

    else:
        #grandeurs normalisées pour observation
        previous_lidar = prev_lid
        current_lidar=(np.concatenate((tableau_lidar_mm[0:101],tableau_lidar_mm[260:360]),axis=None)).astype("float64")/distance_max_lidar_mm
        previous_speed=np.array([float(driver.get_vitesse(obs=True))])
        previous_angle=np.array([float(driver.get_direction(obs=True))])

    observation = {
        "current_lidar": current_lidar,
        "previous_lidar":previous_lidar,
        "previous_speed":previous_speed,
        "previous_angle":previous_angle,
        }
        # print(observation["current_lidar"])
    prev_lid = current_lidar
    return observation, prev_lid

#
#
#
#
# 
# Main Start
# Je mets ca juste pour me retrouver dans le code mdr

# Connexion et démarrage du lidar
lidar = RPLidar("/dev/ttyUSB0",baudrate=256000)
lidar.connect()
print (lidar.get_info())
lidar.start_motor()
time.sleep(1)

tableau_lidar_mm = [0]*360 # Création d'un tableau de 360 zéros (putain ca marche comme ca aussi ?? :thinking: )
prev_lidar = [0]*201

#Load learning data
print("Loading model..")
model = PPO.load("PPO") # Ouais je suis desolé d'avance pour ca
print("model loaded")

driver = Driver()
driver.demarrage_voiture()

SPI_init()
print("SPI Loaded")

print("Appuyer sur 's' pour démarrer")
try :
    time_100_m_s = 0
    etat = 0
    for scan in lidar.iter_scans(scan_type='express') : 
    #Le tableau se remplissant continuement, la boucle est infinie
        #rangement des données dans le tableau
        time_100_m_s += 1
        for i in range(len(scan)) :
            angle = min(359,max(0,359-int(scan[i][1]))) #scan[i][1] : angle 
            tableau_lidar_mm[angle]=scan[i][2]  #scan[i][2] : distance    
        #############################################
        ## Code de conduite (issu du simulateur ou non)
        #############################################
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key_pressed = sys.stdin.readline().rstrip()
            if key_pressed.lower() == "s":
                etat = 1  # Transition to state 1 if 's' is pressed
                
        match etat:
            
            case 0: # Init
                driver.set_vitesse_m_s(0)
                driver.set_direction_degre(0)

            case 1: 
                if tableau_lidar_mm[0] < 160 and tableau_lidar_mm[0] > 0 or tableau_lidar_mm[-30]>0 and tableau_lidar_mm[-30]<160 or tableau_lidar_mm[30]>0 and tableau_lidar_mm[30]<160:
                    driver.set_vitesse_m_s(-driver.get_vitesse_max())
                    etat = 2
                    
                else :
                    obs, prev_lidar = get_observation(prev_lidar)
                    action, _ = model.predict(obs, deterministic=True)

                    consigne_angle = (driver.get_direction(obs=False) + action[1] * 18 * steering_coeff)
                    driver.set_direction_degre(-consigne_angle)
                    consigne_vitesse = (driver.get_vitesse(obs=False) + action[0] * speed_coeff)
                    if consigne_vitesse < 0:
                        consigne_vitesse = 0.1
                    driver.set_vitesse_m_s(consigne_vitesse)

                    # Envoi et interpretation SPI
                    RxBuffer = spi.xfer(TxBuffer)
                    valeur_fourche, valeur_arriere = recompose()
                    if SPI_print == 1: # DEBUG
                        print(TxBuffer)
                        print(valeur_fourche)
                        print(valeur_arriere)
                      
            case 2:
                if (time_100_m_s >= 2):
                    driver.set_vitesse_m_s(0.0)
                    time_100_m_s = 0
                    etat = 3
                    
            case 3:
                if (time_100_m_s) >= 2:
                    time_100_m_s = 0
                if tableau_lidar_mm[0]>0 and tableau_lidar_mm[0]<160:
                    #print("mur devant")
                    driver.set_direction_degre(0)
                    driver.set_vitesse_m_s(-2)
                    
                elif tableau_lidar_mm[-30]>0 and tableau_lidar_mm[-30]<160 :
                    #print("mur à droite")
                    driver.set_direction_degre(-18)
                    driver.set_vitesse_m_s(-2)
                    
                elif tableau_lidar_mm[30]>0 and tableau_lidar_mm[30]<160 :
                    #print("mur à gauche")
                    driver.set_direction_degre(+18)
                    driver.set_vitesse_m_s(-2)
                    
                etat = 4
                    
            case 4:
                if time_100_m_s >= 10:
                    etat = 1
            
        ##############################################
except KeyboardInterrupt: #récupération du CTRL+C
    driver.arret_voiture()
    print("fin des acquisitions")

#arrêt et déconnexion du lidar et des moteurs
lidar.stop_motor()
lidar.stop()
time.sleep(1)
lidar.disconnect()






