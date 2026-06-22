#include <rpcWiFi.h>

const char* ssid = "Leo’s iPhone";
const char* password = "123456789test";

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Test WiFi Wio Terminal");
  Serial.print("Connexion a : ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int essais = 0;

  while (WiFi.status() != WL_CONNECTED && essais < 30) {
    delay(500);
    Serial.print(".");
    essais++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connecte !");
    Serial.print("Adresse IP : ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal RSSI : ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("Echec connexion WiFi");
    Serial.println("Verifie SSID, mot de passe, firmware WiFi et reseau 2.4 GHz.");
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Toujours connecte");
  } else {
    Serial.println("WiFi deconnecte");
  }

  delay(3000);
}