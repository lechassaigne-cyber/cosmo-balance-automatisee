# Projet Balances Cosmo — Contexte pour Claude Code

## C'est quoi ce projet ?

Système IoT de suivi automatisé de l'évaporation de parfums en flacon pour **Cosmo International Fragrances**.
Des balances de précision mesurent en continu le poids des flacons. Les données sont collectées automatiquement, horodatées, et exportées dans un fichier CSV compatible avec le tableau Excel existant des techniciens (15 000 lignes déjà remplies manuellement).

---

## Architecture matérielle

```
Cellule de charge
      ↓
   HX711          ×24 balances (12 câbles RJ45, 2 balances par câble)
      ↓
  Teensy 4.1      lecture HX711, calcul poids, envoi UART
      ↓  (UART Serial1 : TX=pin1, RX=pin0, 115200 baud)
  reTerminal      (Seeed Studio / Raspberry Pi CM4)
      ↓
  fichier CSV     compatible Excel techniciens
```

**Échelle cible :** 2 étagères × 4 niveaux × 12 balances = **96 balances** au total, pilotées par **4 Teensy 4.1** (24 balances chacun). Pour l'instant on développe et teste avec **1 Teensy + 1 balance**.

---

## Câblage HX711 → Teensy

- **SCK commun** : pin 2 (partagé entre tous les HX711)
- **DOUT individuels** : pins 3 à 26 (B1=pin3, B2=pin4, ..., B24=pin27)
- **VCC** : 3.3V du Teensy (important : pas 5V, le Teensy 4.1 n'est pas 5V tolérant)
- **GND** : GND du Teensy

**Câble RJ45 par paire de balances** :

| Fil | Couleur | Signal |
|-----|---------|--------|
| 1 | Marron blanc | GND B1 |
| 2 | Marron | VCC 3.3V B1 |
| 3 | Bleu blanc | SCK B1 |
| 4 | Bleu | DOUT B1 |
| 5 | Vert blanc | GND B2 |
| 6 | Vert | VCC 3.3V B2 |
| 7 | Orange blanc | SCK B2 |
| 8 | Orange | DOUT B2 |

Au panneau central : tous les fils SCK se rejoignent sur **un seul pin 2 du Teensy**.

---

## Fichier Teensy : src/main.cpp

### Ce qu'il fait
- Détecte les HX711 présents au démarrage (timeout 1s par balance)
- Lit les poids en tournant sur toutes les balances (non bloquant)
- Envoie une trame UART toutes les 5 secondes (configurable)
- Balance absente → `ERR` dans la trame
- Stocke tare et facteur de calibration en **EEPROM** (survit aux reboots)

### Format trame envoyée sur Serial1 (→ reTerminal)
```
B1:228,50;B2:ERR;B3:145,30;...;B24:ERR\n
```
- Séparateur décimal : **virgule** (format français, compatible Excel)
- Les lignes commençant par `#` sont des logs/commentaires (ignorés par le collecteur)

### Commandes acceptées depuis Serial1 (← reTerminal)
| Commande | Action |
|----------|--------|
| `TARE:n` | Tare la balance n (1–24) |
| `CALIB:n` | Lance calibration guidée balance n |
| `STATUS` | Liste toutes les balances détectées/absentes |
| `INTERVAL:n` | Change intervalle d'envoi en ms (min 1000) |
| `OK` | Confirmation pendant calibration |

### Pins utilisées
| Pin | Rôle |
|-----|------|
| 0 | RX1 (reçoit commandes reTerminal) |
| 1 | TX1 (envoie trames reTerminal) |
| 2 | SCK commun HX711 |
| 3–27 | DOUT B1–B24 |

---

## Fichier reTerminal : collecteur.py (déjà existant, à modifier)

Le reTerminal tourne sous Linux (Raspberry Pi CM4). Le script Python collecteur.py :
- Écoute Serial1 du Teensy sur `/dev/ttyAMA0` (ou `/dev/ttyS0`) à 115200 baud
- Parse les trames `B1:228,50;B2:ERR;...`
- **À modifier** pour écrire un CSV au format compatible Excel techniciens

### Format CSV cible (compatible Excel existant)

```csv
Experimental code;Measure Request;Méthode;Date de l'action;Commentaire;Masse
B1;Weight;Masse Automatique 001;17/06/2026;;228,50
B2;Weight;Masse Automatique 001;17/06/2026;;ERR
```

- Séparateur de colonnes : **point-virgule** (standard Excel français)
- Date format : `JJ/MM/AAAA`
- Masse : virgule décimale
- Colonne `Commentaire` : vide pour les mesures auto
- Colonne `Méthode` : toujours `Masse Automatique 001` pour les mesures auto

### Table de correspondance balances ↔ flacons

Fichier `config_balances.xlsx` rempli par le technicien sur son PC, déposé sur le reTerminal via SFTP (WinSCP). Format :

```
Balance | Experimental code
B1      | S489-1
B2      | B101-L
...
```

Le collecteur lit ce fichier au démarrage pour renseigner la colonne `Experimental code` dans le CSV.

---

## État d'avancement

### ✅ Fait
- Architecture complète définie (matériel, câblage, protocoles)
- Firmware Teensy `src/main.cpp` écrit et chargé dans VS Code
- `platformio.ini` configuré pour Teensy 4.1 + bibliothèque HX711 bogde

### 🔄 Étape en cours — Étape 1 : test Teensy + 1 balance
- Brancher 1 HX711 sur pins 2 (SCK) et 3 (DOUT), VCC=3.3V, GND
- Flasher via PlatformIO (bouton Upload dans VS Code)
- Ouvrir moniteur série pour vérifier détection et trame

### 📋 Suite prévue
1. ✅ Firmware Teensy (en cours de test)
2. ⬜ Test détection 24 balances
3. ⬜ Modifier collecteur.py reTerminal → nouveau format CSV
4. ⬜ Test communication Teensy ↔ reTerminal + validation CSV
5. ⬜ Alimentation externe pour 24/48/96 balances + 4 Teensy

---

## Environnement de développement

- **VS Code** + extension **PlatformIO IDE**
- **Teensy 4.1** branché en USB sur le PC
- **reTerminal** accessible via VS Code Remote-SSH
- Bibliothèque HX711 : `bogde/HX711@^0.7.5` (déclarée dans platformio.ini, téléchargée auto)

---

## Points d'attention importants

- Le Teensy 4.1 est **3.3V uniquement** — ne jamais mettre 5V sur ses pins GPIO
- Les HX711 doivent être alimentés en **3.3V** (pas 5V) pour que DOUT reste en 3.3V
- SCK **commun** à tous les HX711 — tous reliés au même pin 2
- Les lignes `#` dans la trame UART sont des logs, pas des données — le collecteur doit les ignorer
- Séparateur décimal = **virgule** dans toute la chaîne (Teensy → CSV → Excel)
- Le CSV utilise le **point-virgule** comme séparateur de colonnes (pas la virgule) car Excel français interprète la virgule comme décimale

---

## Bibliothèques utilisées

| Lib | Usage |
|-----|-------|
| `bogde/HX711` | Lecture cellules de charge via HX711 |
| `EEPROM` | Stockage calibration (incluse Arduino/Teensy) |
| `Arduino.h` | Framework de base |