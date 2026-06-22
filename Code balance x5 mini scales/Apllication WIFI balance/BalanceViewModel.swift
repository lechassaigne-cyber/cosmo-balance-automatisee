// BalanceViewModel.swift
// Gère la connexion WiFi, le polling, les alertes et l'export CSV

import Foundation
import Combine
import UserNotifications
import SwiftUI

@MainActor
class BalanceViewModel: ObservableObject {

    // ─── État publié (SwiftUI se met à jour automatiquement) ─────────────────
    @Published var balances: [Balance] = []
    @Published var isConnected: Bool = false
    @Published var lastError: String? = nil
    @Published var lastUpdateTime: Date? = nil
    @Published var config = AppConfig()
    @Published var selectedBalanceID: Int? = nil

    // ─── Privé ───────────────────────────────────────────────────────────────
    private var timer: Timer?
    private var isFirstFetch = true

    // ─── Initialisation ──────────────────────────────────────────────────────
    init() {
        // Initialise 4 balances vides (seront remplies après la première requête)
        for i in 1...4 {
            balances.append(Balance(
                id: i,
                name: "Balance \(i)",
                currentWeight: 0,
                initialWeight: nil,
                alertThreshold: 20.0,  // alerte par défaut à 20g
                history: [],
                isConnected: false
            ))
        }
        requestNotificationPermission()
    }

    // ─── Démarrer / Arrêter le polling ───────────────────────────────────────

    func startPolling() {
        fetchData()  // première requête immédiate
        timer = Timer.scheduledTimer(withTimeInterval: config.pollingInterval, repeats: true) { [weak self] _ in
            Task { await self?.fetchData() }
        }
    }

    func stopPolling() {
        timer?.invalidate()
        timer = nil
    }

    // ─── Requête HTTP vers le Wio Terminal ───────────────────────────────────

    func fetchData() {
        guard let url = URL(string: config.dataURL) else {
            lastError = "URL invalide : \(config.dataURL)"
            return
        }

        var request = URLRequest(url: url)
        request.timeoutInterval = 4.0  // timeout court pour ne pas bloquer l'UI

        URLSession.shared.dataTask(with: request) { [weak self] data, response, error in
            DispatchQueue.main.async {
                if let error = error {
                    self?.isConnected = false
                    self?.lastError = "Connexion impossible : \(error.localizedDescription)"
                    self?.markAllDisconnected()
                    return
                }

                guard let data = data else {
                    self?.lastError = "Aucune donnée reçue"
                    return
                }

                self?.parseAndUpdate(data: data)
            }
        }.resume()
    }

    // ─── Parse le JSON et met à jour les balances ────────────────────────────

    private func parseAndUpdate(data: Data) {
        do {
            let response = try JSONDecoder().decode(WioResponse.self, from: data)
            isConnected = true
            lastError = nil
            lastUpdateTime = Date()

            for balanceData in response.balances {
                updateBalance(id: balanceData.id, name: balanceData.name, weight: balanceData.weight)
            }

            isFirstFetch = false

        } catch {
            lastError = "Erreur de parsing JSON : \(error.localizedDescription)"
            // Pour déboguer, affiche le JSON brut reçu
            if let raw = String(data: data, encoding: .utf8) {
                print("JSON reçu : \(raw)")
            }
        }
    }

    private func updateBalance(id: Int, name: String, weight: Double) {
        guard let index = balances.firstIndex(where: { $0.id == id }) else { return }

        let now = Date()
        var balance = balances[index]

        // Mémorise le poids initial au premier démarrage
        if isFirstFetch {
            balance.initialWeight = weight
        }

        // Calcule le delta par rapport à la mesure précédente
        var delta: Double? = nil
        if let lastPoint = balance.history.last {
            delta = weight - lastPoint.weight
        }

        // Ajoute au historique (garde max 500 points)
        let point = WeightPoint(timestamp: now, weight: weight, delta: delta)
        balance.history.append(point)
        if balance.history.count > 500 {
            balance.history.removeFirst()
        }

        balance.name = name
        balance.currentWeight = weight
        balance.isConnected = true

        // Vérifie l'alerte
        let wasAlerting = balances[index].isAlerting
        balances[index] = balance
        if balance.isAlerting && !wasAlerting {
            sendAlert(for: balance)
        }
    }

    private func markAllDisconnected() {
        for i in balances.indices {
            balances[i].isConnected = false
        }
    }

    // ─── Alertes ─────────────────────────────────────────────────────────────

    private func requestNotificationPermission() {
        UNUserNotificationCenter.current().requestAuthorization(options: [.alert, .sound, .badge]) { _, _ in }
    }

    private func sendAlert(for balance: Balance) {
        let content = UNMutableNotificationContent()
        content.title = "⚠️ Seuil atteint — \(balance.name)"
        content.body = "Poids : \(String(format: "%.2f", balance.currentWeight)) g (seuil : \(String(format: "%.0f", balance.alertThreshold)) g)"
        content.sound = .default

        let request = UNNotificationRequest(
            identifier: "alert-balance-\(balance.id)-\(Date().timeIntervalSince1970)",
            content: content,
            trigger: nil  // immédiat
        )
        UNUserNotificationCenter.current().add(request)
    }

    // ─── Export CSV ──────────────────────────────────────────────────────────

    func exportCSV(for balance: Balance) -> String {
        var csv = "Date,Heure,Poids (g),Delta (g)\n"

        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "dd/MM/yyyy"
        let timeFormatter = DateFormatter()
        timeFormatter.dateFormat = "HH:mm:ss"

        for point in balance.history {
            let date = dateFormatter.string(from: point.timestamp)
            let time = timeFormatter.string(from: point.timestamp)
            let weight = String(format: "%.4f", point.weight)
            let delta = point.delta.map { String(format: "%.4f", $0) } ?? ""
            csv += "\(date),\(time),\(weight),\(delta)\n"
        }
        return csv
    }

    func exportAllCSV() -> String {
        var csv = "Balance,Date,Heure,Poids (g),Delta (g)\n"
        let df = DateFormatter(); df.dateFormat = "dd/MM/yyyy"
        let tf = DateFormatter(); tf.dateFormat = "HH:mm:ss"

        for balance in balances {
            for point in balance.history {
                let w = String(format: "%.4f", point.weight)
                let d = point.delta.map { String(format: "%.4f", $0) } ?? ""
                csv += "\(balance.name),\(df.string(from: point.timestamp)),\(tf.string(from: point.timestamp)),\(w),\(d)\n"
            }
        }
        return csv
    }

    // ─── Paramètres ──────────────────────────────────────────────────────────

    func updateAlertThreshold(for id: Int, threshold: Double) {
        guard let index = balances.firstIndex(where: { $0.id == id }) else { return }
        balances[index].alertThreshold = threshold
    }

    func resetHistory(for id: Int) {
        guard let index = balances.firstIndex(where: { $0.id == id }) else { return }
        balances[index].history = []
        balances[index].initialWeight = balances[index].currentWeight
    }
}
