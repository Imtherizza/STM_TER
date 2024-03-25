# STM_TER
Repository for STML432KC component

Biblios "CoVAPSy" ont été "empruntés" de monsieur Juton :)

# Etat du programme

Le programme emet et recoit des trames d'un maximum de six caractères (6) sur 8bits en SPI \
Le SPI Fonctionne en **Interruption CPU** et j'avoue que pour du SPI vaut mieux passer en DMA mais flemme\
\
L'envoi par défaut programmé est l'envoi de la valeur en cm du télémètre sur les trois premiers caractères d'envoi\
La 4e valeur est 255\
La 5 et 6e valeur sont la fourche optique en dm/s (5 : entier, 6 : décimale)
Un affichage de données est préprogrammé et inclut : 
- Un indicateur de vitesse
- Les données brutes du roulis
- les valeurs captées par le capteur arrière
- Trames de reception SPI (pour ceux qui savent pas utiliser de debugger)

# Activer le SPI
Lorsque le microcontroleur a fini de s'initialiser, appuyer sur le BP2 (proche du RESET jaune) pour activer les transmissions \
Ceci doit être fait de préference avant le démarrage de tout programme sur Raspberry Pi \
**Un programme de test est présent dans le dossier [SPI_Python_Test](https://github.com/Imtherizza/STM_TER/tree/main/SPI_Python_test)** \
\
Reset : Bouton Jaune puis BP 2

# A faire :
- Conduite directe par STM avec toutes les instructions passant par SPI (facultatif)

# Issues :
- La STM recoit 3 bits pour la vitesse? Ce qui rend difficile de distinguer la vraie consigne de vitesse
