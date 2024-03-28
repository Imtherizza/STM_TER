import serial

# Ouvrir le port série COM5
ser = serial.Serial('COM5', 115200)  # Assurez-vous que le débit en bauds correspond à celui configuré sur votre microcontrôleur

try:
    while True:
        # Lire une ligne de données depuis le port série
        data = ser.readline().decode().strip()  # Décodez les données et supprimez les caractères de nouvelle ligne
        print(data)  # Afficher les données reçues
except KeyboardInterrupt:
    # Fermer le port série lorsqu'on appuie sur Ctrl+C pour quitter le programme
    ser.close()