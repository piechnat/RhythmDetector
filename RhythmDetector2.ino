/*****************************************************************************************************************
*   Rhythm Detector II | Układ Blue Pro Micro 5V/16MHz (zgodny z Arduino Leonardo) | (c) 2017 Mateusz Piechnat   *
*****************************************************************************************************************/
#include <EEPROM.h>

const uint8_t 
  MIC_PIN = 2,  // Czujnik dźwięku - cyfrowy 5V (pin z obsługą przerwań)
  BTN_PIN = 3,  // Tact Switch (pin z obsługą przerwań)
  SND_PIN = 9,  // Buzzer z generatorem 5V 
  EEPROM_ID = 0b10101010, /* 170 */
  M_NULL  = 0b000, /* 0 - brak wiadomości */  M_DETECT = 0b001, /* 1 - tryb rozpoznawania */
  M_LEARN = 0b010, /* 2 - tryb uczenia    */  M_SETUP = 0b100;  /* 4 - SETUP zmiany trybu */

struct {
  uint16_t len = 0; // (długość tablicy)
  uint16_t arr[30]; // (tablica)
} beats,            // rozpoznawane wartości rytmiczne (czas trwania między uderzeniami)
  pattern;          // ustalony wzór rytmiczny (proporcje w stosunku do najkrótszego czasu trwania)

volatile struct {   // struktura danych synchronizujących pętlę główną z przerwaniami    
  boolean learnMode = false;        // tryb pracy (true - uczenie, false - rozpoznawanie)
  uint8_t msg = M_SETUP | M_DETECT; // komunikat pętli głównej
  uint32_t param = 0;               // czas uderzenia (pojawienia się dźwięku) w milisekundach
  uint32_t lastParam = 0;           // czas ostatniego uderzenia
  uint32_t lastEvent[2] = {0, 0};   // czas ostatniego zdarzenia (mikrofon i przycisk)
} gblVal;

void beep(byte count) {                   // funkcja sygnałów dźwiękowych
  while (count--) {
    digitalWrite(SND_PIN, HIGH);          // 1 sygnał  - zapisanie wzorca rytmicznego
    delay(70);                            // 2 sygnały - włączenie trybu uczenia
    digitalWrite(SND_PIN, LOW);           // 3 sygnały - rozpoznanie wzorca rytmicznego :)
    delay(200);
  }
}

void onMicNoise() {          // przerwanie - tryb RISING (kiedy tylko pojawi się HIGH)
  uint32_t tick = millis(); 
  if (tick - gblVal.lastEvent[0] > 20) {  // ignorowanie "jakichś" krótkich zmian na pinie sygnałowym czujnika
    gblVal.lastParam = gblVal.param;
    gblVal.param = tick;    
    gblVal.msg = gblVal.learnMode ? M_LEARN : M_DETECT;              
  }                                       // wysłanie komunikatu do pętli głównej
  gblVal.lastEvent[0] = tick;
}

void onBtnPress() {          // przerwanie - tryb LOW (wywoływane bez przerwy do momentu puszczenia przycisku)
  uint32_t tick = millis();               // ignorowanie drgań styków podczas naciskania i puszczania
  if (tick - gblVal.lastEvent[1] > 20) {  // (blokuje również wykonywanie pętli głównej programu)
    gblVal.learnMode = !gblVal.learnMode; // zmiana trybu pracy
    gblVal.msg = M_SETUP | (gblVal.learnMode ? M_LEARN : M_DETECT); 
  }                                       // wysłanie komunikatu do pętli głównej
  gblVal.lastEvent[1] = tick;
}

/****************************************************************************************************************/

void setup() { 
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(MIC_PIN, INPUT); 
  pinMode(SND_PIN, OUTPUT);
  uint8_t x;
  EEPROM.get(0, x);
  if (x == EEPROM_ID) { // odczyt z EEPROM jeśli zgadza się znacznik zapisu
    EEPROM.get(1, pattern); 
    beats.len = pattern.len;
  }
  Serial.begin(9600);
}

void loop() {
  while (gblVal.msg == M_NULL) /* wait */;                           // oczekiwanie na komunikat z przerwania
  gblVal.msg = msgLoop(gblVal.msg, gblVal.param, gblVal.lastParam);  // przetworzenie komunikatu
  if (gblVal.msg != M_NULL) gblVal.learnMode = gblVal.msg & M_LEARN; // (możliwość programowej zmiany trybu)
}

uint8_t msgLoop(uint8_t msg, uint32_t beat, uint32_t lastBeat) {
  uint8_t result = M_NULL;
  if (msg & M_SETUP) { // SETUP
    detachInterrupt(digitalPinToInterrupt(MIC_PIN)); // wyłączenie
    detachInterrupt(digitalPinToInterrupt(BTN_PIN)); // przerwań
    result = (msg & M_LEARN) ? learnSetup() : detectSetup();
    while (digitalRead(BTN_PIN) == LOW); // oczekiwanie na puszczenie przycisku
    delay(20);                           // dodatkowe 20 milisekund na drgania styków
    gblVal.lastParam = gblVal.param = 0; // zerowanie parametrów
    // !!! na Arduino Uno jedno niepotrzebne przerwanie czujnika dźwięku pojawia się natychmiast po użyciu 
    gblVal.lastEvent[0] = gblVal.lastEvent[1] = millis(); // attachInterrupt, dlatego trzeba je zignorować
    attachInterrupt(digitalPinToInterrupt(BTN_PIN), onBtnPress, LOW);    // włączenie
    attachInterrupt(digitalPinToInterrupt(MIC_PIN), onMicNoise, RISING); // przerwań 
  }
  else if (lastBeat) { // LOOP (jeżeli były przynajmniej dwa uderzenia)
    beat -= lastBeat;  // obliczanie czasu trwania wartości rytmicznej
    result = (msg & M_LEARN) ? learnLoop(beat) : detectLoop(beat);
  }
  return result;
}

/****************************************************************************************************************/

uint8_t learnSetup() {         
  beep(2);                     // sygnalizacja trybu uczenia
  pattern.len = beats.len = 0; // zerowanie rozmiaru tablic
  return M_NULL;
}

uint8_t learnLoop(uint16_t beatDuration) {   // TRYB UCZENIA
  pattern.arr[pattern.len++] = beatDuration; // zapis kolejnych wartości rytmicznych
  return (pattern.len < sizeof pattern.arr / sizeof pattern.arr[0]) ? M_NULL : M_SETUP | M_DETECT;
}

uint8_t detectSetup() {                          
  if (pattern.len < 2) return M_SETUP | M_LEARN; // przejście do trybu uczenia jeśli nie ma wzoru
  if (beats.len == 0) {                          // jeżeli ostatnio został zapisany nowy wzór 
    uint32_t lowest = pattern.arr[0], v;         
    for (byte i = 1; i < pattern.len; i++)       // poszukiwanie najkrótszej wartości rytmicznej
      if (pattern.arr[i] < lowest) lowest = pattern.arr[i];
    // zapisywanie wzoru rytmicznego (ustalenie proporcji wartości względem najkrótszej)
    for (byte i = 0; i < pattern.len; i++) 
      pattern.arr[i] = ((v = pattern.arr[i]) * 100) / lowest;
    beats.len = pattern.len;                     // (32bit zmienna "v" do mnożenia przez 100)
    EEPROM.put(0, EEPROM_ID); // zapis do 
    EEPROM.put(1, pattern);   // EEPROM
    beep(1);
  }
  return M_NULL; 
}

uint8_t detectLoop(uint16_t beatDuration) {  // TRYB ROZPOZNAWANIA 
  memcpy(beats.arr, &beats.arr[1], (sizeof beats.arr[0]) * (beats.len - 1)); // przesunięcie tablicy "w lewo"
  beats.arr[beats.len - 1] = beatDuration;   // dopisanie ostatniej wartości "z prawej"
  uint32_t lowest = beats.arr[0], r, v;  
  for (byte i = 1; i < beats.len; i++)       // poszukiwanie najkrótszej wartości
    if (beats.arr[i] < lowest) lowest = beats.arr[i];                        
  for (byte i = 0; i < beats.len; i++) {     // porównywanie wartości (beats) z wzorem (pattern)                               
    r = pattern.arr[i] * 0.25;               // 25% tolerancji błędu
    v = ((v = beats.arr[i]) * 100) / lowest; // przekształcenie wartości do proporcji wzoru rytmicznego
    if ((v < pattern.arr[i] - r) || (v > pattern.arr[i] + r)) return M_NULL; // przerwanie jeśli nie pasuje
  }
  beep(3); // tablica wartości rytmicznych przeszła pomyślną weryfikację :)
  return M_NULL;
}

