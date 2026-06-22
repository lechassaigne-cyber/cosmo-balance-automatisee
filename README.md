# Système automatisé de pesée — Cosmo International Fragrances

> ⚠️ Projet en cours de développement — Stage avril–juillet 2026

**Auteur :** Léo Chassaigne  
**Période :** Avril – Juillet 2026  
**Entreprise :** Cosmo International Fragrances — Mougins (06)  
**Formation :** CESI École d'Ingénieurs — Spécialité Systèmes Électroniques et Électriques Embarqués (S3E)

---

## Contexte

Projet développé dans le cadre d'un stage de deuxième année au sein de la division Technology de Cosmo International Fragrances. L'objectif est de concevoir un système automatisé de pesée permettant de suivre l'évaporation de flacons de parfum au fil du temps, en remplacement des relevés manuels effectués par les équipes du laboratoire.

---

## Historique des versions

### V1 — Python / Mini Unit Scale (I2C)
Première version développée en Python sur Wio Terminal.  
Utilise des balances Mini Unit Scale communicantes via un bus I2C multiplexé (TCA9548A).  
Fonctionnelle sur 4 balances simultanées.

### V2 — Python / HX711
Remplacement des Mini Unit Scale par des cellules de charge associées à des modules HX711.  
Amélioration de la stabilité des mesures.  
Toujours en Python sur Wio Terminal.

### V3 — C++ / HX711 / reTerminal ← version actuelle
Réécriture complète du firmware en C++ sous PlatformIO (framework Arduino).  
Communication BLE Nordic UART Service vers le reTerminal (Raspberry Pi CM4).  
Réception et enregistrement automatique des données en CSV sur le reTerminal.  
Dashboard TFT sur le Wio Terminal (mode OVERVIEW / FOCUS).

---

## Matériel utilisé

| Composant | Rôle |
|---|---|
| Wio Terminal (Seeed Studio) | Microcontrôleur principal, affichage TFT, BLE |
| reTerminal (Raspberry Pi CM4) | Réception BLE, stockage CSV |
| Cellules de charge 1 kg | Mesure du poids |
| Modules HX711 | Amplification signal cellules de charge |
| TCA9548A | Multiplexeur I2C (jusqu'à 8 canaux) |
| Boîtier Fusion 360 | Impression 3D sur mesure |

---

## Architecture actuelle (V3)
Cellules de charge (x4 actuellement)

↓ signal analogique

HX711 (x4)

↓ signal numérique

Wio Terminal (C++ / PlatformIO)

Lecture HX711
Affichage TFT (OVERVIEW / FOCUS)
Émission BLE Nordic UART

↓ Bluetooth

reTerminal (Raspberry Pi CM4)
Réception BLE
Enregistrement CSV horodaté


---

## Fonctionnalités implémentées

- Lecture simultanée de 4 balances HX711
- Affichage TFT : mode OVERVIEW (4 balances) et mode FOCUS (1 balance détaillée)
- Tare et calibration individuelle par masse étalon de 100 g
- Transmission BLE Nordic UART Service vers reTerminal
- Enregistrement automatique en CSV horodaté sur reTerminal

---

## Fonctionnalités en développement

- [ ] Support de 12 balances simultanées (extension multiplexage HX711)
- [ ] Montée en charge vers 20+ balances
- [ ] Interface web de visualisation des données en temps réel
- [ ] Alertes automatiques en cas de dérive anormale

---

## Branchement V3 (4 balances)

| Signal | Broche Wio Terminal |
|---|---|
| SCK commun | D0 |
| DOUT Balance 1 | D1 |
| DOUT Balance 2 | D2 |
| DOUT Balance 3 | D3 |
| DOUT Balance 4 | D4 |
| Alimentation | 3,3 V |
| Masse | GND |

---

## Structure du repo
/v1_python_mini_scales/   → V1 Python + Mini Unit Scale

/v2_cpp_hx711/            → V2 Python + HX711

/v3_reterminal/           → V3 C++ + HX711 + reTerminal (actuelle)

/docs/                    → Schémas, Gantt, documentation

---

## Statut

🟡 En cours — objectif : 12 balances fonctionnelles puis interface web
