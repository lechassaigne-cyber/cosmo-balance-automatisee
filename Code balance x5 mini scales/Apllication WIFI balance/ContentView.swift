// ContentView.swift
// Interface utilisateur complète de l'application

import SwiftUI
import Charts  // nécessite iOS 16+

// ════════════════════════════════════════════════════════════════
// VUE PRINCIPALE — Navigation entre les onglets
// ════════════════════════════════════════════════════════════════

struct ContentView: View {
    @StateObject private var vm = BalanceViewModel()

    var body: some View {
        TabView {
            DashboardView()
                .tabItem { Label("Balances", systemImage: "scalemass") }

            SettingsView()
                .tabItem { Label("Réglages", systemImage: "gearshape") }
        }
        .environmentObject(vm)
        .onAppear { vm.startPolling() }
        .onDisappear { vm.stopPolling() }
    }
}

// ════════════════════════════════════════════════════════════════
// TABLEAU DE BORD — Liste de toutes les balances
// ════════════════════════════════════════════════════════════════

struct DashboardView: View {
    @EnvironmentObject var vm: BalanceViewModel

    var body: some View {
        NavigationStack {
            List {
                // Barre de statut connexion
                Section {
                    ConnectionStatusRow()
                }

                // Liste des 4 balances
                Section("Balances") {
                    ForEach(vm.balances) { balance in
                        NavigationLink(destination: BalanceDetailView(balanceID: balance.id)) {
                            BalanceRowView(balance: balance)
                        }
                    }
                }

                // Export global
                Section {
                    Button(action: exportAll) {
                        Label("Exporter toutes les données (CSV)", systemImage: "square.and.arrow.up")
                    }
                }
            }
            .navigationTitle("Évaporation Parfum")
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button(action: { vm.fetchData() }) {
                        Image(systemName: "arrow.clockwise")
                    }
                }
            }
        }
    }

    private func exportAll() {
        let csv = vm.exportAllCSV()
        let av = UIActivityViewController(activityItems: [csv], applicationActivities: nil)
        if let scene = UIApplication.shared.connectedScenes.first as? UIWindowScene,
           let vc = scene.windows.first?.rootViewController {
            vc.present(av, animated: true)
        }
    }
}

// ────────────────────────────────────────────────────────────────
// Barre de statut de connexion
// ────────────────────────────────────────────────────────────────

struct ConnectionStatusRow: View {
    @EnvironmentObject var vm: BalanceViewModel

    var body: some View {
        HStack {
            Circle()
                .fill(vm.isConnected ? Color.green : Color.red)
                .frame(width: 10, height: 10)

            VStack(alignment: .leading, spacing: 2) {
                Text(vm.isConnected ? "Connecté" : "Non connecté")
                    .font(.subheadline)
                    .fontWeight(.medium)

                if let error = vm.lastError {
                    Text(error)
                        .font(.caption)
                        .foregroundStyle(.secondary)
                } else if let date = vm.lastUpdateTime {
                    Text("Dernière mise à jour : \(date.formatted(date: .omitted, time: .standard))")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }
        }
        .padding(.vertical, 4)
    }
}

// ────────────────────────────────────────────────────────────────
// Ligne d'une balance dans la liste
// ────────────────────────────────────────────────────────────────

struct BalanceRowView: View {
    let balance: Balance

    var body: some View {
        HStack {
            // Indicateur de couleur
            RoundedRectangle(cornerRadius: 3)
                .fill(balance.statusColor)
                .frame(width: 4, height: 44)

            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Text(balance.name)
                        .font(.headline)
                    Spacer()
                    if balance.isAlerting {
                        Label("Seuil", systemImage: "exclamationmark.triangle.fill")
                            .font(.caption)
                            .foregroundStyle(.red)
                    }
                }

                HStack {
                    Text(String(format: "%.2f g", balance.currentWeight))
                        .font(.title2)
                        .fontWeight(.semibold)
                        .monospacedDigit()

                    Spacer()

                    VStack(alignment: .trailing, spacing: 2) {
                        Text(String(format: "▼ %.4f g/h", balance.evaporationRate))
                            .font(.caption)
                            .foregroundStyle(.secondary)
                        Text(String(format: "−%.2f g total", balance.totalLoss))
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }
            }
        }
    }
}

// ════════════════════════════════════════════════════════════════
// DÉTAIL D'UNE BALANCE
// ════════════════════════════════════════════════════════════════

struct BalanceDetailView: View {
    @EnvironmentObject var vm: BalanceViewModel
    let balanceID: Int

    @State private var showExportSheet = false

    private var balance: Balance? {
        vm.balances.first(where: { $0.id == balanceID })
    }

    var body: some View {
        Group {
            if let b = balance {
                balanceContent(b)
            } else {
                Text("Balance introuvable").foregroundStyle(.secondary)
            }
        }
    }

    @ViewBuilder
    func balanceContent(_ b: Balance) -> some View {
        List {
            // ─── Poids actuel ───────────────────────────────────────────
            Section {
                VStack(alignment: .leading, spacing: 8) {
                    Text("Poids actuel")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    HStack(alignment: .firstTextBaseline) {
                        Text(String(format: "%.4f", b.currentWeight))
                            .font(.largeTitle)
                            .fontWeight(.bold)
                            .monospacedDigit()
                        Text("g")
                            .font(.title2)
                            .foregroundStyle(.secondary)
                    }
                    if b.isAlerting {
                        Label("Seuil d'alerte atteint", systemImage: "exclamationmark.triangle.fill")
                            .foregroundStyle(.red)
                            .font(.subheadline)
                    }
                }
                .padding(.vertical, 4)
            }

            // ─── Métriques ──────────────────────────────────────────────
            Section("Métriques") {
                MetricRow(label: "Poids initial", value: b.initialWeight.map { String(format: "%.2f g", $0) } ?? "—")
                MetricRow(label: "Perte totale", value: String(format: "%.2f g", b.totalLoss))
                MetricRow(label: "Taux d'évaporation", value: String(format: "%.4f g/h", b.evaporationRate))
                MetricRow(label: "Nombre de mesures", value: "\(b.history.count)")
            }

            // ─── Graphique ──────────────────────────────────────────────
            if !b.history.isEmpty {
                Section("Évolution du poids") {
                    WeightChartView(history: b.history)
                        .frame(height: 200)
                        .padding(.vertical, 8)
                }
            }

            // ─── Seuil d'alerte ─────────────────────────────────────────
            Section("Alerte") {
                AlertThresholdView(balanceID: b.id, threshold: b.alertThreshold)
            }

            // ─── Historique récent ──────────────────────────────────────
            Section("Dernières mesures") {
                ForEach(b.history.suffix(20).reversed()) { point in
                    HStack {
                        Text(point.timestamp, format: .dateTime.hour().minute().second())
                            .font(.caption)
                            .foregroundStyle(.secondary)
                            .monospacedDigit()
                        Spacer()
                        Text(String(format: "%.4f g", point.weight))
                            .font(.caption)
                            .monospacedDigit()
                        if let d = point.delta {
                            Text(String(format: "%+.4f", d))
                                .font(.caption)
                                .foregroundStyle(d < 0 ? .red : .green)
                                .monospacedDigit()
                        }
                    }
                }
            }

            // ─── Actions ────────────────────────────────────────────────
            Section {
                Button("Exporter en CSV") {
                    let csv = vm.exportCSV(for: b)
                    let av = UIActivityViewController(activityItems: [csv], applicationActivities: nil)
                    if let scene = UIApplication.shared.connectedScenes.first as? UIWindowScene,
                       let vc = scene.windows.first?.rootViewController {
                        vc.present(av, animated: true)
                    }
                }

                Button("Réinitialiser l'historique", role: .destructive) {
                    vm.resetHistory(for: b.id)
                }
            }
        }
        .navigationTitle(b.name)
        .navigationBarTitleDisplayMode(.inline)
    }
}

// ────────────────────────────────────────────────────────────────
// Ligne de métrique
// ────────────────────────────────────────────────────────────────

struct MetricRow: View {
    let label: String
    let value: String

    var body: some View {
        HStack {
            Text(label).foregroundStyle(.secondary)
            Spacer()
            Text(value).fontWeight(.medium).monospacedDigit()
        }
    }
}

// ────────────────────────────────────────────────────────────────
// Graphique d'évolution du poids (Swift Charts — iOS 16+)
// ────────────────────────────────────────────────────────────────

struct WeightChartView: View {
    let history: [WeightPoint]

    // N'affiche que les 200 derniers points pour la performance
    private var displayPoints: [WeightPoint] {
        Array(history.suffix(200))
    }

    var body: some View {
        Chart(displayPoints) { point in
            LineMark(
                x: .value("Temps", point.timestamp),
                y: .value("Poids (g)", point.weight)
            )
            .foregroundStyle(Color(red: 0, green: 0.78, blue: 0.64))
            .interpolationMethod(.catmullRom)

            AreaMark(
                x: .value("Temps", point.timestamp),
                y: .value("Poids (g)", point.weight)
            )
            .foregroundStyle(
                LinearGradient(
                    colors: [Color(red: 0, green: 0.78, blue: 0.64).opacity(0.3), .clear],
                    startPoint: .top,
                    endPoint: .bottom
                )
            )
            .interpolationMethod(.catmullRom)
        }
        .chartXAxis {
            AxisMarks(values: .automatic(desiredCount: 4)) { value in
                AxisValueLabel(format: .dateTime.hour().minute())
            }
        }
        .chartYAxis {
            AxisMarks(values: .automatic(desiredCount: 4)) { value in
                AxisValueLabel { if let v = value.as(Double.self) { Text(String(format: "%.1f g", v)) } }
            }
        }
    }
}

// ────────────────────────────────────────────────────────────────
// Réglage du seuil d'alerte
// ────────────────────────────────────────────────────────────────

struct AlertThresholdView: View {
    @EnvironmentObject var vm: BalanceViewModel
    let balanceID: Int
    @State var threshold: Double

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Label("Seuil d'alerte", systemImage: "bell.badge")
                Spacer()
                Text(String(format: "%.0f g", threshold))
                    .fontWeight(.semibold)
            }
            Slider(value: $threshold, in: 1...200, step: 1) {
                Text("Seuil")
            } minimumValueLabel: {
                Text("1g").font(.caption)
            } maximumValueLabel: {
                Text("200g").font(.caption)
            }
            .onChange(of: threshold) { _, newVal in
                vm.updateAlertThreshold(for: balanceID, threshold: newVal)
            }
        }
    }
}

// ════════════════════════════════════════════════════════════════
// RÉGLAGES — Configuration de la connexion WiFi
// ════════════════════════════════════════════════════════════════

struct SettingsView: View {
    @EnvironmentObject var vm: BalanceViewModel
    @State private var ipInput: String = ""
    @State private var portInput: String = "80"
    @State private var intervalInput: String = "5"

    var body: some View {
        NavigationStack {
            Form {
                Section("Adresse du Wio Terminal") {
                    HStack {
                        Text("IP")
                        TextField("ex: 192.168.1.100", text: $ipInput)
                            .keyboardType(.numbersAndPunctuation)
                            .autocorrectionDisabled()
                            .multilineTextAlignment(.trailing)
                    }
                    HStack {
                        Text("Port")
                        TextField("80", text: $portInput)
                            .keyboardType(.numberPad)
                            .multilineTextAlignment(.trailing)
                    }
                    Button("Appliquer") {
                        vm.config.wioIP = ipInput
                        vm.config.wioPort = Int(portInput) ?? 80
                        vm.stopPolling()
                        vm.startPolling()
                    }
                    .disabled(ipInput.isEmpty)
                }

                Section("Polling") {
                    HStack {
                        Text("Intervalle")
                        Spacer()
                        TextField("5", text: $intervalInput)
                            .keyboardType(.decimalPad)
                            .frame(width: 60)
                            .multilineTextAlignment(.trailing)
                        Text("sec")
                    }
                    Button("Appliquer") {
                        vm.config.pollingInterval = Double(intervalInput) ?? 5.0
                        vm.stopPolling()
                        vm.startPolling()
                    }
                }

                Section("Statut") {
                    HStack {
                        Text("URL actuelle")
                        Spacer()
                        Text(vm.config.dataURL)
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                    Button("Tester la connexion") {
                        vm.fetchData()
                    }
                }

                Section("Format JSON attendu") {
                    Text("""
{
  "timestamp": 1714300000,
  "balances": [
    {"id": 1, "name": "Flacon A", "weight": 48.32},
    {"id": 2, "name": "Flacon B", "weight": 62.15},
    {"id": 3, "name": "Flacon C", "weight": 23.89},
    {"id": 4, "name": "Flacon D", "weight": 55.10}
  ]
}
""")
                    .font(.system(.caption, design: .monospaced))
                    .foregroundStyle(.secondary)
                }
            }
            .navigationTitle("Réglages")
            .onAppear {
                ipInput = vm.config.wioIP
                portInput = "\(vm.config.wioPort)"
                intervalInput = "\(Int(vm.config.pollingInterval))"
            }
        }
    }
}

// ════════════════════════════════════════════════════════════════
// PREVIEW (uniquement pour Xcode)
// ════════════════════════════════════════════════════════════════

#Preview {
    ContentView()
}
