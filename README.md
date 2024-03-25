# STM_TER
Repository for STML432KC component

Biblios "CoVAPSy" ont été "empruntés" de monsieur Juton :)

# Etat du programme

Le programme emet et recoit des trames d'un maximum de six caractères (6) sur 8bits en SPI \
Le SPI Fonctionne en **Interruption CPU** et j'avoue que pour du SPI vaut mieux passer en DMA mais flemme\
\
L'envoi par défaut programmé est l'envoi de la valeur en cm du télémètre sur les six caractères d'envoi\
Un affichage de données est préprogrammé et inclut : 
- Un indicateur de vitesse
- Les données brutes du roulis
- les valeurs captées par le capteur arrière
- Trames de reception SPI (pour ceux qui savent pas utiliser de debugger)

# Activer le SPI
Lorsque le microcontroleur a fini de s'initialiser, appuyer sur le BP2 (proche du RESET jaune) pour activer les transmissions \
Ceci doit être fait de préference avant le démarrage du programme sur Raspberry Pi \
**Un programme de test est présent dans le dossier [SPI_Python_Test](https://github.com/Imtherizza/STM_TER/tree/main/SPI_Python_test)** 

# A faire :
- Conduite directe par STM avec toutes les instructions passant par SPI (facultatif)
- Fourche optique?
