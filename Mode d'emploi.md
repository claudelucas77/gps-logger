# gps-logger
gps


Mode d'emploi : 




Affichage :
SD en haut à gauche signifie que la carte SD est présente et bien prise en compte
fin. ou vit. donne le type de retour son (finesse ou vitesse) (version bluetooth et retour son)
B si le bluetooth est actif (version bluetooth et retour son)
vit xx.x nous donne la vitesse (version bluetooth et retour son)
en gros au centre, c'est l'altitude GPS en mètres
sat x : le bombre de satellites vus
en bas à gauche, un petit "o" clignote quand l'enregistrement est en cours
en haut à droite, le fichier en cours d'enregistrement : il s'affiche au moment du début de l'enregistrement


un BP marche permet de lancer l'enregistrement. il FAUT le couper avant d'éteindre le logger, sinon le fichier est perdu.
On sait si c'est le cas si le "o" clignote en bas à gauche, c'est que ca enregistre



  version avec bluetooth  :
  
  
Le bluetooth de l'esp32 va etre activé à la mise sous tnesion pendant un certain temps (2 min)

Pendant ce laps de temps on peut se connecter avec le telephone (appli Serial Bluetooth Terminal)

Il faut connecter le tel au bluetooth de l'appareil (nom = ESP 32 A)

Différentes commandes sont disponibles, visibles en tapant "help"
help      :    Aide" = affiche l'aide
logon     :    Activ logs" = Active les logs
logoff    :    Désactiv logs" = desactive les logs
altixxxx  :    xxxx = Altitude d'ouverture pour bip" = Altitude pour altison (pas encore testé) 0= pas d'altison
finesse   :    Retour son sur la finesse" 
vitesse   :    Retour son sur la vitesse" 
btoff     :    Désactiver Bluetooth" = on s'en fout, il va se désactiver tout seul
volx      :    Volume son (1-8)" = règle le volume du retour son
rec       :    Lancer enregistrement" = lance l'enregistrement sur la carte SD (remplace le bouton)
stop      :    Arrêter enregistrement" = Arrete l'enregistrement sur la SD (remplace le bouton)


Le bluetooth sert au paramétrage. Ne vous amusez pas avec juste avant de sauter car CA PLANTE avec la comm bluetooth.

Si c'est le cas, l'écran s'éteint. Il faudra le redémarrer.

Les paramètres restent mémorisés pour le volume, retour son, et altitude de l'altison 
