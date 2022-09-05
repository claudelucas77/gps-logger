////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//                                                    CABLAGE

////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Puce GPS VK2828U7G5LF
// Fil rouge et fil jaune du GPS : Cabler sur le 3.3V
// Fil noir du GPS : Cabler sur GND
// Fil bleu du GPS : Cabler sur RX (pin26) de l'ESP32 (Attention il faut croiser les RX/TX en série)
// Fil vert du GPS : Cabler sur TX (pin27) de l'ESP32 (Attention il faut croiser les RX/TX en série)
// Fil blanc du GPS : On s'en fout, on s en sert pas

// Prise jack
// Masse de la jack : Cabler sur GND
// Ecouteur gauche ou droit : Cabler sur la pin 12 et GND (Attention : Brancher l'un ou l'autre, ou refaire un calcul d'impédance pour voir si ca crame pas la sortie)

// BP enregistrement : pin 19 et GND



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

//                                                     BIBLIOTHEQUES

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

    #include <HardwareSerial.h>     /// Bibliothèque Serial
    #include <Adafruit_GPS.h>       /// Bibliotheque ADAFRUIT pour le GPS
    #include <Adafruit_SSD1331.h>   //// Afficheur
    #include <EEPROM.h>             /// Librairie pour copie EEPROM (memoire morte)
    
    #include <SPI.h>                /// Pour datalogging (Utilisation SPI)
    #include <SD.h>                 /// Pour datalogging
    
    


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

//                                                      VARIABLES

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    


// Interruption cyclique        (Copier coller d'un code trouvé sur internet)
    hw_timer_t * timer = NULL;

// Variables pour le GPS

    HardwareSerial mySerial(1);       // Instance du port série
    Adafruit_GPS GPS(&mySerial);      // Instance de la librairie GPS

// Afficheur
    #define sclk 14
    #define mosi 13
    #define cs   15
    #define rst  4
    #define dc   16
    
    // Color definitions
    #define BLACK           0x0000
    #define BLUE            0x001F
    #define RED             0xF800
    #define GREEN           0x07E0
    #define CYAN            0x07FF
    #define MAGENTA         0xF81F
    #define YELLOW          0xFFE0  
    #define WHITE           0xFFFF       


/// Carte SD
    SPIClass SPI2(HSPI);
    File dataFile;                                           /// Variable fichier
    String nom_fichier ;                                     /// Nom du fichier

/// Afficheur
    Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);   ///Déclaration variable pour afficheur (TTGO en SPI 96x64) 




/// Autres variables
    bool Def_Aff = 0;                                        /// Défauts matériel Afficheur

    bool fm500ms = 0;                                        /// Front montant 500ms
    bool fm200ms = 0;                                        /// Front montant 200ms
    bool cligno = 0;                                         /// Bit clignotant 500ms

    double VitVerticale = 0 ;                                 /// Vitesse verticale
    double VitHoriz_m_s ;                                     /// Conversion vitesse GPS en m/s
    double finesse = 0 ;                                      /// Finesse 
    double vit_totale = 0;                                     /// Vitesse totale (sur axe)
    double vitN = 0;
    double vitE = 0;

    float prev_altitude=0;                                    /// Valeur précédente d'altitude
    int prev_milliseconds = 0;                                /// Valuer précédente de temps (partie ms)


    bool REC_SD = false;                                      /// Enregistrement sur carte SD      
    bool sd_ok = 0;                                           /// Carte SD est en place et ok        

    bool BP_rec = 0;                                          /// BP qui déclenche l'enregistrement
    bool BP_rec_filtre = 0;                                   /// BP filtré à 50ms sur la montée
    int BP_rec_tempo = millis() ;                                     /// Tempo de filtrage rebond du BP
    bool pre_BP_rec_filtre = 0;                               /// Valeur précédente du BP (pour construction front montant)
    bool FM_BP_rec = 0;                                       /// Front montant d'un appui BP

    byte curseur=0;                                           /// Pour tache rapide
    char ligne[85];                                           /// Contient la ligne reçue en cours de remplissage  
    bool new_line1=false;                                     /// Indicateur nouvelle ligne arrivée RMC
    bool new_line2=false;                                     /// Indicateur nouvelle ligne arrivée CGA
    char ligne_new1[85];                                       /// Ligne terminée stockée jusqu'à nouvelle valeur RMC
    char ligne_new2[85];                                       /// Ligne terminée stockée jusqu'à nouvelle valeur CGA
    bool fichier_defini = false;                              /// Nom de Fichier défini
    int temp_debug = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

//                                                  TACHE RAPIDE

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

void IRAM_ATTR onTimer() {            /// La tache rapide ne sert qu'à lire les données qui arrivent du GPS

      char c;
     
      while (mySerial.available()) {
        
          c=mySerial.read();
          
          ligne[curseur]=c;
          
          curseur++;
          
          if (c==13) { ///retour chariot (fin de ligne)
            
                      ///Remplissage ligne par caractères vides
                      for (byte i=curseur ; i<=84 ; i++) {
                        ligne[i]=0;
                      }
                                 
                       //// remise a 0 curseur pour la ligne
                       curseur=0;


                        if (ligne[4]=='R' ){ ///Trame RMC

                             ///Copie char ligne vers ligne_new
                             for (int j=0 ; j<84 ; j++) {
                             ligne_new1[j]=ligne[j];
                             }
                                                                         
                             new_line1=true;
                        }

                        if (ligne[4]=='G' ){ ///Trame CGA

                             ///Copie char ligne vers ligne_new
                             for (int j=0 ; j<84 ; j++) {
                             ligne_new2[j]=ligne[j];
                             }
                                                                         
                             new_line2=true;
                        }
                                                   
                      }
         
          
           }


  
}
 






/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

//                                                    SETUP

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

void setup() {

/// Paramétrage de l'interruption cyclique
  timer = timerBegin(0, 80, true);                /// 80 est un prédiviseur sur 80Mhz
  timerAttachInterrupt(timer, &onTimer, true);    
  timerAlarmWrite(timer, 100000, true);           /// 100000 est 80Mhz / 80 qui s'est produit 100 000 fois (1ms)
  timerAlarmEnable(timer);                        /// Activation alarme


/// Bouton REC
  pinMode(19,INPUT_PULLUP);                                        /// Entrée du BP qui lance l'enregistrement


/// Ports séries
  mySerial.begin(9600, SERIAL_8N1, 26, 27); // Vitesse, format, pinRX, pinTX : Pour le GPS
  delay(1000);                             // Timer 1s pour attendre ouverture des ports


//////////////////////////////// Paramétrage de la puce GPS     ////////////////////////////////////////////////////////////////
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
//  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);

/// SWITCH TO 5HZ
byte a[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, 0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30}; // -> 5HZ

/// DISABLE COMMANDS
byte b[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x44, 0x54, 0x4d, 0x2a, 0x33, 0x42, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x0a, 0x00, 0x04, 0x23}; // -> Disable GPDTM
byte c[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x42, 0x53, 0x2a, 0x33, 0x30, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x09, 0x00, 0x03, 0x21}; // -> Disable GPGBS
byte d[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x47, 0x41, 0x2a, 0x32, 0x37, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x00, 0x00, 0xfa, 0x0f}; // -> Disable GPGGA
byte e[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x4c, 0x4c, 0x2a, 0x32, 0x31, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x01, 0x00, 0xfb, 0x11}; // -> Disable GPGLL
byte f[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x52, 0x53, 0x2a, 0x32, 0x30, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x06, 0x00, 0x00, 0x1b}; // -> Disable GPGRS
byte g[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x53, 0x41, 0x2a, 0x33, 0x33, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x02, 0x00, 0xfc, 0x13}; // -> Disable GPGSA
byte h[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x53, 0x54, 0x2a, 0x32, 0x36, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x07, 0x00, 0x01, 0x1d}; // -> Disable GPGST
byte i[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x53, 0x56, 0x2a, 0x32, 0x34, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x03, 0x00, 0xfd, 0x15}; // -> Disable GPGSV
byte j[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x52, 0x4d, 0x43, 0x2a, 0x33, 0x41, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x04, 0x00, 0xfe, 0x17}; // -> Disable GPRMC
byte k[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x56, 0x54, 0x47, 0x2a, 0x32, 0x33, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x05, 0x00, 0xff, 0x19}; // -> Disable GPVTG
byte l[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x5a, 0x44, 0x41, 0x2a, 0x33, 0x39, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x08, 0x00, 0x02, 0x1f}; // -> Disable GPZDA

/// ENABLE COMMANDS RMC et GGA
byte m[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x52, 0x4d, 0x43, 0x2a, 0x33, 0x41, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x04, 0x01, 0xff, 0x18 }; //-> Enable GPRMC
byte n[] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x47, 0x41, 0x2a, 0x32, 0x37, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x00, 0x01, 0xfb, 0x10 }; //-> Enable GPGGA

mySerial.write(a,sizeof(a));
mySerial.write(b,sizeof(b));
mySerial.write(c,sizeof(c));
//mySerial.write(d,sizeof(d));
mySerial.write(e,sizeof(e));
mySerial.write(f,sizeof(f));
mySerial.write(g,sizeof(g));
mySerial.write(h,sizeof(h));
mySerial.write(i,sizeof(i));
//mySerial.write(j,sizeof(j));
mySerial.write(k,sizeof(k));
mySerial.write(l,sizeof(l));
mySerial.write(m,sizeof(m));
mySerial.write(n,sizeof(n));




/// Afficheur
  display.begin() ;                                         /// Initialise l'afficheur (Addresse : 0x3C, dimensions 128x32)
  display.fillScreen(BLACK);


/// Carte SD
  SPI2.begin(18, 19, 23, 5);                                 /// Specifique TTGO port SPI n°2
  if (!SD.begin(5, SPI2)) {                                  /// Initialisation carte SD
  
    } else {
    
    sd_ok = true;
    }



/// Fix GPS
  pinMode(32, INPUT);                                        /// Pin du fix GPS


  REC_SD=false ;                                             /// Met à 0 l'enregistrement


}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

//                                                BOUCLE PRINCIPALE

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

void loop() {

/// debug
if (FM_BP_rec){

  temp_debug++;
}


/// Mémorisation de l'horloge lors de la mise sous tension  
static int compteur = millis();


/// Fonctions d'horloge
  horloge();
  
 

/// Commandes depuis la console BT ou série
  commandes_console();                    /// Ne contient plus que le bouton d'enregistrement                                 

                                                          
  //// BP Rec (FM et anti rebond)
  FM_bouton();


/// Traitement GPS
  fonction_GPS();


//// Calcul Vitesse verticale toutes les 200ms
  if (fm200ms) calcul_vit_verticale();


/// Calcul Vitesse totale (Pythagore) & Finesse //
  
  VitHoriz_m_s = GPS.speed * 0.5144;
  vit_totale = sqrt( (VitHoriz_m_s * VitHoriz_m_s) + (VitVerticale * VitVerticale));    //  en m/s

  if (VitVerticale > 0 | VitVerticale < 0) {
      finesse = VitHoriz_m_s / abs(VitVerticale);
  } else {
      finesse = 500;
  }

///// AFFICHAGE   //////    
  affichage();






}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

//                                                  FONCTIONS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

void affichage() {


    if (fm500ms) {                                                                                    /// Affichage des infos sur l'intervalle
  
  
  afficher(1,12,"     ",1);
  afficher(1,12,String(temp_debug),1);

      
        /// Infos afficheur OLED
      afficher(50, 50, "sat",1);  
      afficher(75, 50, "   ", 1) ;                                                                    /// Vers afficheur
      afficher(75, 50, String(GPS.satellites), 1) ;                                                   /// Vers afficheur
      
      afficher(30, 30, "     ", 2) ;                                                                  /// Vers afficheur
      afficher(30, 30, String(GPS.altitude,0), 2) ;                                                   /// Vers afficheur
      afficher(90, 30, "m", 1) ;                                                                      /// Vers afficheur
      

         
      afficher(1,1,"  ",1);
      if (sd_ok) {
        afficher(1,1,"SD OK",1);                                                                      /// Affiche SD si la sd est ok
      }else{
        afficher(1,1,"SD NOK",1);
      }
      
     
      afficher(1,50," ",1);
      if (REC_SD && cligno ) afficher(1,50,"o",1);                                                    /// Si enregistrement en cours le o clignote



     if (fichier_defini) {                                                                            /// Affichage nom du fichier en cours d'écriture
      afficher(50,1,"       ",1);
      afficher(50,1, nom_fichier.c_str() ,1);
      
     }
           
    }


  
}


///////////////////////////   AFFICHER DU TEXTE     ///////////////////////////////

void afficher(int x, int y, String txt, int taille) {

    
    display.setTextSize(taille);
    int longueur = txt.length();
    display.setTextColor(WHITE);
    display.fillRect(x, y, taille*6*txt.length(), 8*taille, BLACK); /// x,y,largeur,hauteur,couleur
    display.setCursor(x, y);
    display.println(txt);
      
}

//// test des chaines de caractères sur xxx caractères
bool test_char(char chaine1[], char chaine2[], int nb_car) {

  bool result = true;

  for (int i=0 ; i < nb_car ; i++) {

    if (chaine1[i] != chaine2[i]) {
       result = false;
    }
  
  }

  return result;
}


/////////   CALCUL VITESSE VERTICALE   /////
void calcul_vit_verticale() {
  
static int pre_altitude;

  VitVerticale = (GPS.altitude - pre_altitude) / 0.2;        // la valeur doit etre positive pour une descente (correspondance flysight)          

  pre_altitude = GPS.altitude;
}


void horloge() {            

static int tps_500ms ;
static int tps_200ms ;

  fm500ms=0;
  fm200ms=0;
  
  if (millis() > tps_500ms + 500) {
      tps_500ms = millis();
      fm500ms=1;
      cligno = ! cligno;
  }
  if (millis() > tps_200ms + 200) {
      tps_200ms = millis();
      fm200ms=1;
  }
  
}







void commandes_console() {



//// Appui BP REC
if (millis()>5000) {                /// Sur probleme d'alimentation, l'ESP se coupe et se rallume en début de démarrage. La tempo permet d'ignorer l'etat du BP pendant 5s le temps que ca se fasse (il est en NC)
  
      if (FM_BP_rec) {
      REC_SD = !REC_SD;
      }


  
      }else{                      //// Tant qu'on a pas 5s d'écoulés, on force l'enregistrement à 0
        REC_SD=0;
      }

}


void archivage() {


String ligne;

        ligne+=(F("20"));
        ligne+=(GPS.year);
        ligne+=(F("-"));
        ligne+=(format(GPS.month));
        ligne+=(F("-"));
        ligne+=(format(GPS.day));
        ligne+=(F("T"));
        ligne+=(format(GPS.hour));
        ligne+=(F(":"));
        ligne+=(format(GPS.minute));
        ligne+=(F(":"));
        ligne+=(format(GPS.seconds));
        ligne+=(F("."));
        ligne+=(GPS.milliseconds);
        ligne+=(F("Z,"));
        ligne+=(String(GPS.latitudeDegrees,6));
        ligne+=(F(","));
        ligne+=(String(GPS.longitudeDegrees,6));
        ligne+=(F(","));
        ligne+=(String(GPS.altitude,4));
        ligne+=(F(","));
        ligne+=(String(vitN,4));
        ligne+=(F(","));
        ligne+=(String(vitE,4));
        ligne+=(F(","));
        ligne+=(String(VitVerticale,4));
        ligne+=(F(","));
        ligne+=(String(GPS.HDOP,4));
        ligne+=(F(",3.0,0.5,"));
        ligne+=(GPS.angle);
        ligne+=(F(",5.0,3,"));
        ligne+=(GPS.satellites);

//GPS.HDOP
    
    
    

if (REC_SD) {

  if (!fichier_defini) {                                              /// Si début d'enregistrement, on définit un nom de fichier libre
      
      for (int i=1; i<100 ; i++) {                                    /// On scanne jusqu'à temps qu'un fichier soit inexistant
            nom_fichier = "/" ;
            nom_fichier += i ;
            nom_fichier += ".csv";          
            if (!SD.exists(nom_fichier.c_str())) {
              break;
            }
      }

      
      fichier_defini = true;
      
      dataFile = SD.open(nom_fichier.c_str(), FILE_WRITE);                                                       /// Ouverture fichier
            

      dataFile.println(F("time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,heading,cAcc,gpsFix,numSV"));
      dataFile.println(F(",(deg),(deg),(m),(m/s),(m/s),(m/s),(m),(m),(m/s),(deg),(deg),,"));
            
      
    }     /// Fin création nouveau fichier


        

  
/// Concaténation de la ligne à remplir dans le fichier        
  if (dataFile) {

        
        dataFile.println(ligne);
               
  }

  
  } else {

      dataFile.close();                                                                                     /// Fermeture du fichier
      fichier_defini=false; 
  }

}     /// Fin archivage()




void FM_bouton() {

    
    BP_rec = !digitalRead(19);
    
    if (!BP_rec) BP_rec_tempo = millis();               //// RAZ tempo de rebond si BP n'est pas appuyé


    
      if (BP_rec && millis() > BP_rec_tempo + 100) {
        
          BP_rec_filtre = 1;
         
      } else {
      
          BP_rec_filtre = 0;
      }
      
      /// Front montant
      if (BP_rec_filtre && !pre_BP_rec_filtre) {
          
          FM_BP_rec = 1;
        
      } else {
      
          FM_BP_rec = 0;
      }
      
        pre_BP_rec_filtre = BP_rec_filtre;
     
}
  

void fonction_GPS() {            ///////////////////   GPS   ////////////////////

  if (new_line1 & new_line2) {
  
    
    GPS.parse(ligne_new1);                                                             /// Analyse trame (bibliotheque GPS Adafruit)
    GPS.parse(ligne_new2);                                                             /// Analyse trame (bibliotheque GPS Adafruit)



    vitN = VitHoriz_m_s * cos(radians(GPS.angle));                                      /// Calcul vitesse Nord
    vitE = VitHoriz_m_s * sin(radians(GPS.angle));                                      /// Calcul vitesse Est


/// Calcul vitesse verticale
   
        VitVerticale = (prev_altitude - GPS.altitude) / 0.2;   /// Valeurs toutes les 0.2s           
        prev_altitude = GPS.altitude;
        

/// Lancement archivage dans le fichier


/// Lorsque on a une trame RMC correcte on archive les valeurs  ///
 
     archivage();                /// Archive dans le fichier les nouvelles données
  
  
/// Fin de nouvelle donnée
    new_line1=false;
    new_line2=false;
 
  }    

}


///////////////   Fonction mise en forme d'un int vers string ///////

String format(byte nb) {
String chaine="";

if (nb<10){
  chaine+="0";
  }
chaine+=nb;

return(chaine);

}
