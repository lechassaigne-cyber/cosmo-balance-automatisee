# Guide d'installation — App Balances Parfum

## Ce que tu as reçu
- `BalancesApp.swift`   — point d'entrée de l'app
- `Models.swift`        — structures de données
- `BalanceViewModel.swift` — logique WiFi + polling + alertes
- `ContentView.swift`   — toute l'interface utilisateur

---

## Étape 1 — Installer Xcode (Mac uniquement)

1. Ouvre le **Mac App Store**
2. Cherche **Xcode** et installe-le (gratuit, ~10 Go)
3. Lance Xcode une première fois pour qu'il installe ses composants

---

## Étape 2 — Créer un projet Xcode

1. Lance Xcode → **Create New Project**
2. Choisis : **iOS → App**
3. Remplis :
   - **Product Name** : `BalancesParfum`
   - **Interface** : SwiftUI
   - **Language** : Swift
   - **Team** : sélectionne ton Apple ID (gratuit !)
   - **Bundle Identifier** : `com.tonnom.balancesparfum`
4. Clique **Next** → choisis un dossier → **Create**

---

## Étape 3 — Ajouter les fichiers Swift

1. Dans Xcode, dans le panneau gauche tu vois un dossier `BalancesParfum`
2. **Supprime** le `ContentView.swift` existant (clic droit → Delete → Move to Trash)
3. Fais **glisser-déposer** tes 4 fichiers `.swift` dans ce dossier dans Xcode
4. Xcode te demande : coche **"Copy items if needed"** → **Finish**

---

## Étape 4 — Configurer iOS 16 minimum

Swift Charts nécessite iOS 16+.

1. Clique sur le projet (ligne bleue tout en haut dans le panneau gauche)
2. Onglet **General**
3. Rubrique **Minimum Deployments** → change en **iOS 16.0**

---

## Étape 5 — Autoriser HTTP (non-HTTPS)

Par défaut iOS bloque les connexions HTTP. Il faut l'autoriser pour ton réseau local.

1. Dans Xcode, ouvre le fichier **Info.plist**
2. Clic droit dans la liste → **Add Row**
3. Clé : `NSAppTransportSecurity` (type : Dictionary)
4. Dans ce dictionnaire, ajoute : `NSAllowsLocalNetworking` = **YES**

Ou directement en XML dans Info.plist :
```xml
<key>NSAppTransportSecurity</key>
<dict>
    <key>NSAllowsLocalNetworking</key>
    <true/>
</dict>
```

---

## Étape 6 — Lancer sur ton iPhone (GRATUIT)

1. Branche ton iPhone en USB
2. Sur l'iPhone : **Réglages → Confidentialité & sécurité → Mode développeur → Activer**
3. Dans Xcode en haut : sélectionne ton iPhone dans la liste des appareils
4. Clique ▶ (le bouton Play)
5. La première fois, Xcode te demande d'aller dans **Réglages iPhone → VPN et gestion des appareils** → fais confiance à ton certificat

---

## Étape 7 — Connecter l'app à ton Wio Terminal

1. Lance l'app sur ton iPhone
2. Va dans l'onglet **Réglages**
3. Entre l'adresse IP de ton Wio Terminal (ex: `192.168.1.100`)
4. Port : `80`
5. Clique **Appliquer** puis **Tester la connexion**

### Pour trouver l'IP du Wio Terminal :
Dans ton code Arduino/Wio, ajoute cette ligne et regarde le Serial Monitor :
```cpp
Serial.println(WiFi.localIP());
```

---

## Format JSON que le Wio Terminal doit renvoyer

Ton Wio Terminal doit servir un endpoint HTTP GET `/data` qui renvoie :

```json
{
  "timestamp": 1714300000,
  "balances": [
    {"id": 1, "name": "Flacon A", "weight": 48.32},
    {"id": 2, "name": "Flacon B", "weight": 62.15},
    {"id": 3, "name": "Flacon C", "weight": 23.89},
    {"id": 4, "name": "Flacon D", "weight": 55.10}
  ]
}
```

### Exemple minimal de code Wio Terminal (Arduino) :

```cpp
#include <WiFi.h>
#include <WebServer.h>
#include "HX711.h"

const char* ssid = "TON_WIFI";
const char* password = "TON_MOT_DE_PASSE";

WebServer server(80);
HX711 scales[4];  // configure tes 4 balances

// Pins DATA et CLK pour chaque HX711
// Adapte selon ton câblage !
const int DATA_PINS[] = {2, 4, 6, 8};
const int CLK_PINS[]  = {3, 5, 7, 9};

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.println(WiFi.localIP());  // ← note cette IP pour l'app

  // Initialise les 4 HX711
  for (int i = 0; i < 4; i++) {
    scales[i].begin(DATA_PINS[i], CLK_PINS[i]);
    scales[i].set_scale(/* ton facteur de calibration */);
    scales[i].tare();
  }

  // Route HTTP GET /data
  server.on("/data", HTTP_GET, []() {
    String json = "{\"timestamp\":" + String(millis()/1000) + ",\"balances\":[";
    const char* names[] = {"Flacon A","Flacon B","Flacon C","Flacon D"};
    for (int i = 0; i < 4; i++) {
      float w = scales[i].get_units(5);  // moyenne sur 5 lectures
      json += "{\"id\":" + String(i+1) + ",\"name\":\"" + names[i] + "\",\"weight\":" + String(w, 4) + "}";
      if (i < 3) json += ",";
    }
    json += "]}";
    server.send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  server.handleClient();
}
```

---

## Résumé des fonctionnalités de l'app

- ✅ Lecture temps réel des 4 balances via WiFi
- ✅ Graphique d'évolution du poids dans le temps
- ✅ Calcul automatique du taux d'évaporation (g/h)
- ✅ Alertes push quand un seuil est dépassé
- ✅ Export CSV par balance ou global
- ✅ Historique des mesures (jusqu'à 500 points par balance)
- ✅ Configuration IP/port depuis l'app
- ✅ Test gratuit sur iPhone avec Xcode (pas besoin de compte payant)
