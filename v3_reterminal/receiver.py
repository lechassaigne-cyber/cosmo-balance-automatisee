# ============================================================
# reTerminal — Récepteur Serial + Enregistrement CSV
# Version V1.0
#
# Reçoit les données du Wio Terminal via USB Serial
# Format attendu : DATA,<b1>,<b2>,<b3>,<b4>
# Exemple        : DATA,48.72,32.15,0.00,0.00
#
# Les données sont enregistrées dans un fichier CSV horodaté :
# mesures_evaporation.csv
#
# UTILISATION :
#   python3 receiver.py
#
# DÉPENDANCES :
#   pip3 install pyserial
# ============================================================

import serial
import csv
import os
from datetime import datetime

# ============================================================
# CONFIGURATION
# ============================================================

SERIAL_PORT    = "/dev/ttyACM0"  # Port USB du Wio Terminal
BAUD_RATE      = 115200
CSV_FILE       = "mesures_evaporation.csv"
NB_BALANCES    = 4

# ============================================================
# INITIALISATION CSV
# ============================================================

def init_csv(filepath):
    """Crée le fichier CSV avec en-têtes si inexistant."""
    if not os.path.exists(filepath):
        with open(filepath, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "date", "heure",
                "balance_1_g", "balance_2_g",
                "balance_3_g", "balance_4_g"
            ])
        print(f"Fichier CSV créé : {filepath}")
    else:
        print(f"Fichier CSV existant utilisé : {filepath}")

# ============================================================
# ENREGISTREMENT D'UNE LIGNE
# ============================================================

def save_measurement(filepath, weights):
    """Ajoute une ligne de mesure dans le CSV."""
    now = datetime.now()
    row = [now.strftime("%Y-%m-%d"), now.strftime("%H:%M:%S")] + weights
    with open(filepath, mode='a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(row)
    print(f"[{now.strftime('%Y-%m-%d %H:%M:%S')}] Enregistré : {weights}")

# ============================================================
# PARSING DE LA LIGNE SERIAL
# ============================================================

def parse_data_line(line):
    """
    Parse une ligne DATA,b1,b2,b3,b4
    Retourne une liste de floats ou None si format invalide.
    """
    line = line.strip()
    if not line.startswith("DATA,"):
        return None

    parts = line.split(",")
    if len(parts) != NB_BALANCES + 1:
        return None

    try:
        weights = [float(p) for p in parts[1:]]
        return weights
    except ValueError:
        return None

# ============================================================
# BOUCLE PRINCIPALE
# ============================================================

def main():
    init_csv(CSV_FILE)

    print(f"Connexion au port {SERIAL_PORT} à {BAUD_RATE} baud...")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
        print("Connexion établie. En attente de données...")
    except serial.SerialException as e:
        print(f"Erreur ouverture port Serial : {e}")
        print("Vérifiez que le Wio Terminal est bien branché sur le port USB.")
        return

    try:
        while True:
            raw_line = ser.readline()
            if not raw_line:
                continue

            try:
                line = raw_line.decode("utf-8")
            except UnicodeDecodeError:
                continue

            weights = parse_data_line(line)
            if weights is not None:
                save_measurement(CSV_FILE, weights)
            else:
                # Affiche les autres messages du Wio (status, debug)
                print(f"[WIO] {line.strip()}")

    except KeyboardInterrupt:
        print("\nArrêt du programme.")
    finally:
        ser.close()
        print("Port Serial fermé.")

if __name__ == "__main__":
    main()
