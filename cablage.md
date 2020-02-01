# gps-logger
gps


Cablage :

L esp32 est capable de gérer une batterie, mais le composant qui le fait nécessite un appui sur un bp reset pour qu il démarre.

solutions :

- Mettre un bp poussoir et appuyer dessus a chaque fois qu on met l interrupteur sur on

- mettre un autre composant pour la charge et la régulation de tension et alimenter l esp32 et le gps via la borne 3.3V
(Dommage car on ne profite pas des composants de la carte et plus de câblage et soudure)

- solution que j' ai utilisé :
  J alimente le tout avec une batterie lipo. La tension est bien pour le gps mais un peu élevée pour l esp32
  
  Interrupteur sur on:
  j alimente l esp32 derrière une diode qui fait perde 0.7v sur la borne 3.3
  J alimente le gps avec la batterie directement
  L alimentation batterie de l esp32 est relié a un interrupteur spécifique pour la charge

  Interrupteur sur charge:
  Lorsque on charge la batterie on se met sur la position de l interrupteur qui relie la batterie sur l alimentation normale de l esp32


  Interrupteur sur off
  Le + de la batterie n est relié a rien

  Un interrupteur 3 positions est nécessaire dans ce cas pour faire on/off/charge. On peut se débrouille avec 2 interrupteurs sinon

