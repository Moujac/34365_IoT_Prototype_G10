//
//  UserData.swift
//  BlindCare
//
//  Created by Nipun Fernando on 12/11/2025.
//

import Foundation
import Observation
import UserNotifications

// -------------------------------------------------------------------
// MARK: - UserData Class
// -------------------------------------------------------------------

/// Manages the application state, including device location and alert status via polling.
@Observable
final class UserData {
    
    // MARK: - State Properties
    
    /// The ID of the device being monitored.
    var deviceId: String = "N/A"
    
    /// Current latitude received from the updates.
    var currentLatitude: Double = 0.0
    
    /// Current longitude received from the updates.
    var currentLongitude: Double = 0.0
    
    /// The current alert status (true/false) from the latest snapshot.
    var currentAlertStatus: Bool = false
    
    /// Timestamp of the last received update.
    var lastUpdated: String = "N/A"
    
    /// Tracks if the background polling loop is currently running.
    var isPollingActive: Bool = false
    
    var currentAddress: String = "No data..."

    // MARK: - Task Management
    
    /// The Task handle for the background GPS polling loop, returned from Networking.swift.
    private var gpsPollingTask: Task<Void, Never>?

    // MARK: - Initializer & Setup
    
    init() {
        print("UserData initialized.")
    }
    
    // MARK: - Public Methods
    
    /**
     * Executes a single fetch to populate the initial state when the app launches.
     */
    func loadInitialSnapshot() async {
        print("Loading initial snapshot...")
        if let snapshot = await fetchLatestSnapshot() {
            // Use updateState to ensure the sticky logic is applied even to the initial load
            await MainActor.run {
                self.updateState(with: snapshot)
            }
        }
    }
    
    /**
     * Starts the continuous polling loop by utilizing the function in Networking.swift.
     */
    func startMonitoringGPS() {
        // Only start if not already active
        guard !isPollingActive else { return }
        
        self.isPollingActive = true
        
        // Start the polling task, providing the update handler
        self.gpsPollingTask = startGPSPolling { [weak self] snapshot in
            // This closure runs on the MainActor, thanks to Networking.swift
            self?.updateState(with: snapshot)
        }
    }
    
    /**
     * Stops the continuous polling loop by cancelling the task handle.
     */
    func stopMonitoringGPS() {

        gpsPollingTask?.cancel()
        gpsPollingTask = nil
        self.isPollingActive = false 
    }
    
    /**
     * Sends the "01" acknowledgment payload to the server.
     * If successful, the local alert status is immediately resolved.
     */
    func sendDistressAcknowledgement() async -> Bool {
        let success = await sendAcknowledgement(payload: "01")
        if success {
        }
        return success
    }

    /**
     * Resets the local UI state to clear the alert and map markers.
     * This is only called after a successful acknowledgment POST request.
     */
    func resolveDistressSignalLocally() {
        self.currentLatitude = 0.0
        self.currentLongitude = 0.0
        self.currentAlertStatus = false // Clear the sticky state
        self.lastUpdated = "Resolved Locally by User"
        self.currentAddress = "No data..."
        
        UNUserNotificationCenter.current().setBadgeCount(0)

    }

    // MARK: - Private Methods
    
    private func updateState(with snapshot: DeviceSnapshot) {

        self.deviceId = snapshot.deviceId
        self.lastUpdated = snapshot.receivedAt

        if snapshot.alert || self.currentAlertStatus == false {
            self.currentLatitude = snapshot.latitude
            self.currentLongitude = snapshot.longitude
        }
        
        

        if snapshot.alert {
            
            if self.currentAlertStatus == false {
                sendLocalAlertNotification()
            }
            
            self.currentAlertStatus = true
            self.currentLatitude = snapshot.latitude
            self.currentLongitude = snapshot.longitude
        }
        
    }
    
    //Send local notification to the user, when alert status is true
    private func sendLocalAlertNotification() {
        let content = UNMutableNotificationContent()
        content.title = "⚠️ Distress Alert Detected"
        content.body = "Your smart cane has sent a distress signal."
        content.sound = .default
        content.badge = 1

        let request = UNNotificationRequest(
            identifier: UUID().uuidString,
            content: content,
            trigger: nil
        )

        UNUserNotificationCenter.current().add(request) { error in
            if let error = error {
                print("Error showing notification: \(error)")
            } else {
                print("Distress notification delivered.")
            }
        }
    }

}


