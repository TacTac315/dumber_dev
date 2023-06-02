
### Équipe
Ayoub Belafki
Vincent Eychenne
Jérémy Marceau
Dimitri Pizzinat

#### Fonctionnalités demandées

 1. Reprise de la communication entre le Superviseur et le Moniteur après la perte (_fonctionnalités 5-6_)
 - Non implémenté
 2. Compteur à trois (_fonctionnalités 8-9_)
 - Fonctionne, mais nous oublions peut-être de l'appeler dans certaines tâches
 3. Watchdog (_fonctionnalité 11_)
 - Fonctionne, mais le robot ne démarrage que si le démarrage avec watchdog est coché dans le moniteur
 4. Batterie (_fonctionnalité 13_)
 - Fonctionne, mais tout le temps. Il faudrait ajouter un sémaphore afin de ne lancer la tâche que si le niveau de batterie est demandé par le moniteur.
 5. Caméra (envoi périodique + open/close) (_fonctionnalités 14-15-16_)
 - Fonctionne
 6. Arène (_fonctionnalité 17_)
 - Fonctionne
 7. Position  (_fonctionnalité 18-19_)
 - Fonctionne

# Dumber

  

Depot du projet de temps reel 4eme année au departement GEI de l'INSA Toulouse.

  

## Repertoires

- hardware : contient les plans pour la partie mecanique du robot et de son chargeur, ainsi que les plans de conception des PCB du robot, du chargeur, de l'adaptateur Xbee pour la raspberry et les plans des CAP du robot

- software: rassemble les parties logicielles du robot, du chargeur, les bibliotheques et superviseur coté raspberry et l'interface Web

- doc: contient les sujets de TD et TP

- aruco_markers: Script de generation des tags (aruco) utilisés sur les robots