#include "rpcWiFi.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);

  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("Demarrage scan WiFi...");
}

void loop() {
  Serial.println("Scan en cours...");

  int n = WiFi.scanNetworks();

  if (n == 0) {
    Serial.println("Aucun reseau trouve");
  } else {
    Serial.print(n);
    Serial.println(" reseaux trouves :");

    for (int i = 0; i < n; i++) {
      Serial.print(i + 1);
      Serial.print(" : ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" | Signal RSSI : ");
      Serial.println(WiFi.RSSI(i));
    }
  }

  Serial.println("--------------------");
  delay(5000);
}