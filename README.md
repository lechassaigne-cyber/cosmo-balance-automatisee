# Système automatisé de pesée — Cosmo International Fragrances

**Auteur :** Léo Chassaigne  
**Période :** Avril – Juillet 2026  
**Entreprise :** Cosmo International Fragrances — Mougins (06)  
**Formation :** CESI École d'Ingénieurs — Spécialité Sytèmes Électroniques et Électriques Embarqués (S3E) 

---

## Contexte

Ce projet a été développé dans le cadre d'un stage de deuxième année au sein 
de la division Technology de Cosmo International Fragrances. L'objectif était 
de concevoir un système automatisé de pesée permettant de suivre l'évaporation 
de flacons de parfum au fil du temps, en remplacement des relevés manuels 
effectués par les équipes du laboratoire.

---

## Architecture du projet

### V1 — Python + Mini Unit Scale (I2C)
Première version du système, développée en Python sur Wio Terminal.  
Utilise des balances Mini Unit Scale communicantes via un bus I2C (TCA9548A).  
Fichier : `Balance HX711 mode OVERVIEW & FOCUS 4 balances avec meme horloge.py`

### V2 — C++ + Cellules de charge HX711 + reTerminal
Version finale développée en C++ sous PlatformIO (Framework Arduino).  
Remplace les Mini Unit Scale par des cellules de charge associées à des modules 
HX711 pour une meilleure stabilité des mesures (variations < 0,2 g).  
Communication via USB Serial vers un reTerminal (Raspberry Pi).  
Fichiers : `main.cpp`, `platformio.ini`

### V3 — Script reTerminal (Raspberry Pi)
Script Python tournant sur le reTerminal.
Reçoit les données Serial du Wio Terminal (format DATA,b1,b2,b3,b4)
et les enregistre automatiquement dans un fichier CSV horodaté.
Fichier : `v3_reterminal/receiver.py`

---

## Matériel utilisé

- Wio Terminal (Seeed Studio)
- Cellules de charge 1 kg
- Modules amplificateurs HX711
- reTerminal (Raspberry Pi CM4)
- Boîtier sur mesure modélisé sous Fusion 360 et imprimé en 3D

---

## Branchement (V2)

| Signal | Broche Wio Terminal |
|--------|-------------------|
| SCK commun | D0 |
| DOUT Balance 1 | D1 |
| DOUT Balance 2 | D2 |
| DOUT Balance 3 | D3 |
| DOUT Balance 4 | D4 |
| Alimentation | 3,3 V |
| Masse | GND |

---

## Fonctionnalités

- Mode **OVERVIEW** : affichage simultané des 4 balances
- Mode **FOCUS** : affichage détaillé d'une balance (tare, calibration)
- Sauvegarde automatique en fichier CSV toutes les 6 heures
- Envoi des données via Serial vers le reTerminal
- Calibration individuelle par masse étalon de 100 g
