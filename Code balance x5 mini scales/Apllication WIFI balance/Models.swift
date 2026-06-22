// Models.swift
// Structures de données de l'application

import Foundation
import SwiftUI

// ─── Réponse JSON du Wio Terminal ───────────────────────────────────────────
// Le Wio Terminal doit renvoyer exactement ce format JSON sur GET /data
// Exemple : {"timestamp":1714300000,"balances":[{"id":1,"name":"Flacon A","weight":48.32},...]}

struct WioResponse: Codable {
    let timestamp: Double
    let balances: [BalanceData]
}

struct BalanceData: Codable {
    let id: Int
    let name: String
    let weight: Double  // en grammes
}

// ─── Modèle interne de l'application ────────────────────────────────────────

struct Balance: Identifiable {
    let id: Int
    var name: String
    var currentWeight: Double       // poids actuel en grammes
    var initialWeight: Double?      // poids au début de l'expérience
    var alertThreshold: Double      // alerte si poids < seuil
    var history: [WeightPoint]      // historique des mesures
    var isConnected: Bool

    // Perte totale depuis le début
    var totalLoss: Double {
        guard let initial = initialWeight else { return 0 }
        return max(0, initial - currentWeight)
    }

    // Taux d'évaporation moyen (g/heure) sur les dernières mesures
    var evaporationRate: Double {
        guard history.count >= 2 else { return 0 }
        let recent = history.suffix(12)  // 12 dernières mesures
        guard let first = recent.first, let last = recent.last else { return 0 }
        let deltaTime = last.timestamp.timeIntervalSince(first.timestamp) / 3600  // en heures
        let deltaWeight = first.weight - last.weight
        guard deltaTime > 0 else { return 0 }
        return deltaWeight / deltaTime
    }

    // Alerte active si poids en dessous du seuil
    var isAlerting: Bool {
        return currentWeight < alertThreshold && alertThreshold > 0
    }

    // Couleur de statut
    var statusColor: Color {
        if !isConnected { return .gray }
        if isAlerting { return .red }
        return Color(red: 0, green: 0.78, blue: 0.64)  // vert-teal
    }
}

// Un point de mesure dans le temps
struct WeightPoint: Identifiable {
    let id = UUID()
    let timestamp: Date
    let weight: Double

    // Variation par rapport à la mesure précédente (nil pour la première)
    var delta: Double?
}

// ─── Configuration de connexion ──────────────────────────────────────────────

struct AppConfig {
    // ⚠️ IMPORTANT : remplace par l'adresse IP de ton Wio Terminal
    // Pour la trouver : dans ton code Arduino, affiche l'IP dans Serial Monitor
    // Exemple : Serial.println(WiFi.localIP());
    var wioIP: String = "192.168.1.100"
    var wioPort: Int = 80
    var pollingInterval: Double = 5.0  // secondes entre chaque requête

    var baseURL: String {
        return "http://\(wioIP):\(wioPort)"
    }

    var dataURL: String {
        return "\(baseURL)/data"
    }
}
