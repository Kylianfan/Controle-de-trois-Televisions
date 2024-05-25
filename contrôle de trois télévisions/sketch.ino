#include <LiquidCrystal_I2C.h> // Inclusion de la bibliothèque LiquidCrystal_I2C

// Seuil de luminosité pour autoriser l'allumage des télévisions
#define SEUIL_LUMINOSITE 500

// Déclaration des pins pour les capteurs et actionneurs
#define LUMINOSITY_SENSOR_PIN 34
#define MOTION_SENSOR_PIN 13
// I2C LCD display addresses
const int LCD1_ADDRESS = 0x27; // Replace with the actual I2C address of LCD 1
const int LCD2_ADDRESS = 0x28; // Replace with the actual I2C address of LCD 2
const int LCD3_ADDRESS = 0x29; // Replace with the actual I2C address of LCD 3

const long interval = 5000; // Intervalle de 2 heures en millisecondes


// Déclaration des timers
TimerHandle_t screenOffTimer1;
TimerHandle_t screenOffTimer2;


// Déclaration des objets LiquidCrystal_I2C pour chaque écran LCD
LiquidCrystal_I2C lcd1(LCD1_ADDRESS, 16, 2);
LiquidCrystal_I2C lcd2(LCD2_ADDRESS, 16, 2);
LiquidCrystal_I2C lcd3(LCD3_ADDRESS, 16, 2);

// Déclaration de la file de messages pour les événements de luminosité
QueueHandle_t luminosityEventQueue;
QueueHandle_t luminosityEventQueue2;
QueueHandle_t motionEventQueue1;
QueueHandle_t motionEventQueue2;
// Déclaration de la sémaphore multiple pour contrôler l'accès aux écrans
SemaphoreHandle_t tvSemaphore;

// Variable pour suivre l'état des télévisions
bool tv1_is_on = false;
bool tv2_is_on = false;
bool tv3_is_on = false;



// Fonction de rappel pour éteindre l'écran
void turnOffScreen1(TimerHandle_t xTimer) {
  lcd1.clear();
  lcd1.noBacklight();
  tv1_is_on = false;
  xSemaphoreGive(tvSemaphore);

}



// Tâche de gestion de la luminosité
void taskLuminosity(void *pvParameters) {
  int luminosity;
  while (1) {
    // Lire la valeur du capteur de luminosité
    luminosity = analogRead(LUMINOSITY_SENSOR_PIN);
    
    // These constants should match the photoresistor's "gamma" and "rl10" attributes
    const float GAMMA = 0.7;
    const float RL10 = 50;
    float voltage = luminosity / 1024. * 5;
    float resistance = 2000 * voltage / (1 - voltage / 5);
    float luminosity = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));
    // Afficher la valeur de luminosité sur le moniteur série
    Serial.print("Luminosity: ");
    Serial.println(luminosity);
    // Contrôler l'allumage/éteignage des écrans en fonction de la luminosité
    if (luminosity >= SEUIL_LUMINOSITE) {
      // Envoyer un événement de luminosité à la file de messages
      // on peut également utiliser d'autres mécanismes
      //comme les évenements ... ( à vous de choisir)
      xQueueSend(luminosityEventQueue, &luminosity, portMAX_DELAY);
    }
    else{
      lcd1.init();
      lcd1.clear();
      lcd1.noBacklight();
     // Variable pour suivre l'état de la télévision 1
      tv1_is_on = false;

      lcd2.init();
      lcd2.clear();
      lcd2.noBacklight();
      tv2_is_on = false;

      lcd3.init();
      lcd3.clear();
      lcd3.noBacklight();
      tv3_is_on = false;


    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Attendre 1 seconde avant de vérifier à nouveau
  }
}

// Tâche de gestion de lecran en cas d'urgence si télé allumé
void taskTelevision2(void *pvParameters) {
  int luminosity;
  while (1) {
    // Attendre un événement de luminosité
    if (xQueueReceive(luminosityEventQueue, &luminosity, portMAX_DELAY) == pdPASS) {
        vTaskDelay((1000/ portTICK_PERIOD_MS));
   
      if (xSemaphoreTake(tvSemaphore, portMAX_DELAY) == pdTRUE) {
        lcd2.init();
        lcd2.backlight();
        lcd2.setCursor(0, 0);
        lcd2.print("TV2");
        tv2_is_on = true; // Marquer la télévision 1 comme allumée

        // on active le timer respecter la contrainte de temps (deadline)
        xTimerStart(screenOffTimer1, 0);    
        vTaskDelay((5000 / portTICK_PERIOD_MS)); // Attendre 1 seconde avant de vérifier à nouveau
       // Démarrer le timer pour éteindre l'écran après 5 secondes
       lcd2.clear();
       lcd2.noBacklight();
       tv2_is_on=false;
       xSemaphoreGive(tvSemaphore);
       Serial.print("TOKEN RENDU 2 ");
      }
    }
  }
}

// Tâche de gestion de lecran en cas d'urgence si télé allumé
void taskTelevision1(void *pvParameters) {
  int luminosity;
  while (1) {
    // Attendre un événement de luminosité
    if (xQueueReceive(luminosityEventQueue, &luminosity, portMAX_DELAY) == pdPASS) {
      vTaskDelay((1000/ portTICK_PERIOD_MS));
   
      if (xSemaphoreTake(tvSemaphore, portMAX_DELAY) == pdTRUE) {
        lcd3.init();
        lcd3.backlight();
        lcd3.setCursor(0, 0);
        lcd3.print("TV3");
        tv3_is_on = true; // Marquer la télévision 1 comme allumée

        // on active le timer respecter la contrainte de temps (deadline)
        xTimerStart(screenOffTimer1, 0);    
        vTaskDelay((5000 / portTICK_PERIOD_MS)); // Attendre 1 seconde avant de vérifier à nouveau
       // Démarrer le timer pour éteindre l'écran après 5 secondes
        lcd3.clear();
         lcd3.noBacklight();
         tv3_is_on=false;
         xSemaphoreGive(tvSemaphore);
           Serial.print("TOKEN RENDU 3 ");
      }
    }
  }
}


// Tâche de gestion de lecran en cas d'urgence si télé allumé
void taskTelevision3(void *pvParameters) {
  int luminosity;
  while (1) {
    // Attendre un événement de luminosité
    if (xQueueReceive(luminosityEventQueue, &luminosity, portMAX_DELAY) == pdPASS) {
      vTaskDelay((1000/ portTICK_PERIOD_MS));
   
      if (xSemaphoreTake(tvSemaphore, portMAX_DELAY) == pdTRUE) {
        // allumer écran 1
        lcd1.init();
        lcd1.backlight();
        lcd1.setCursor(0, 0);
        lcd1.print("TV1");
        tv1_is_on = true; // Marquer la télévision 1 comme allumée

        // on active le timer respecter la contrainte de temps (deadline)
        xTimerStart(screenOffTimer1, 0);    
        vTaskDelay((5000 / portTICK_PERIOD_MS)); // Attendre 1 seconde avant de vérifier à nouveau
       // Démarrer le timer pour éteindre l'écran après 5 secondes
         lcd1.clear();
         lcd1.noBacklight();
         tv1_is_on=false;
         xSemaphoreGive(tvSemaphore);
         Serial.print("TOKEN RENDU 1 ");
      }
    }
  }
}



// Routine de service d'interruption pour le capteur de mouvement
void motionSensorISR() {

// Fonction doit être très cours car elle va intérompre le système du coup, 
//l'idéee c'est que ISR ici va juste envoyer un signal à d'autres fonctions
//prioritaire (taches normal faute prioriter) pour eviter de prednre trop de temps
// sinon si vous metteez un long code dans cette fonction ISR, le systèems
//risque d'être interrompu un long temps

//on envoie un signal a travers la queue rapidement et on sort de 
// la fonction ISR, les taches ayant besoin de ce signal 
// devront etre programméer avec une priorité max afin 
  int danger;
  if (tv1_is_on) { xQueueSend(motionEventQueue2, &danger, portMAX_DELAY);}
   else {xQueueSend(motionEventQueue1, &danger, portMAX_DELAY);}
 

}

// Nouvelle tâche pour gérer les événements de mouvement
void taskTelevision1_case1(void *pvParameters) {
  int danger;
  while (1) {
    // Attendre un message ou un événement de la file de messages ou de la queue de tâches
    if (xQueueReceive(motionEventQueue1, &danger, portMAX_DELAY) == pdPASS) {
      Serial.println("Motion Detected ! Ecran 1 était dans l'état éteint");
      Serial.println(danger);
      lcd1.init(); // initialiser l'écran
      lcd1.backlight(); //allumer
      lcd1.setCursor(0, 0);
      lcd1.print("Message d'urgence!"); // Afficher le message d'urgence
      xTimerStart(screenOffTimer2, 0); 
    }
  }
}

// Nouvelle tâche pour gérer les événements de mouvement
void taskTelevision1_case2(void *pvParameters) {
  int danger;
  while (1) {
    // Attendre un message ou un événement de la file de messages ou de la queue de tâches
    if (xQueueReceive(motionEventQueue2, &danger, portMAX_DELAY) == pdPASS) {
      Serial.println("Motion Detected !Ecran 1 était dans l'état allumer");
      //ici on pourrais ce demander pk ne pas juste éteindre puis allumer
      //l'écran. Ici dans notre cas, faire ceci marche effiectivemet,
      // car éteintre et allujmé un ecran cela ne va pas couter 
      //chère en cou^t brut. Mais dans certains sysstème,
      // faire une telle action peut couter enormement 
      // et générer d'enorme perte. Le but ici est de sensibiliser
      // a cela. 
      Serial.println(danger);
      lcd1.clear(); // Cette commande efface le contenu de l'écran LCD au lieu d'etteindre l'ecran et de le rallumer
      lcd1.setCursor(0, 0);
      lcd1.print("Message d'urgence!"); // Afficher le message d'urgence
      xTimerStart(screenOffTimer2, 0);
    }
  }
}

void setup() {
  Serial.begin(9600);
  // Initialisation des pins
  pinMode(LUMINOSITY_SENSOR_PIN, INPUT);
  pinMode(MOTION_SENSOR_PIN, INPUT);

  // Création de la file de messages pour les événements de luminosité
  luminosityEventQueue = xQueueCreate(5, sizeof(int));
  motionEventQueue1 = xQueueCreate(5, sizeof(int));
  motionEventQueue2 = xQueueCreate(5, sizeof(int));

  // Création de la sémaphore multiple pour contrôler l'accès aux écrans
  tvSemaphore = xSemaphoreCreateCounting(2, 2);

  screenOffTimer1 = xTimerCreate("Timer Ecran 1", pdMS_TO_TICKS(5000), pdFALSE, 0, turnOffScreen1);
  screenOffTimer2 = xTimerCreate("Timer Ecran 1 urgence", pdMS_TO_TICKS(1000), pdFALSE, 0, turnOffScreen1);
  // Attacher l'ISR au capteur de mouvement
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motionSensorISR, CHANGE);

  // Démarrage et configuration des tâches
  xTaskCreatePinnedToCore(taskLuminosity, "Lire Luminosité", 1500, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskTelevision1, "Allumer télé 1", 1500, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskTelevision2, "Allumer télé 2", 1500, NULL, 1, NULL, 1);
   xTaskCreatePinnedToCore(taskTelevision3, "Allumer télé 3", 1500, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskTelevision1_case1, "Allumer télé 1 cas 1", 1500, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(taskTelevision1_case2, "Allumer télé 1 cas 2", 1500, NULL, 4, NULL, 1);

  

}



void loop() {
}
