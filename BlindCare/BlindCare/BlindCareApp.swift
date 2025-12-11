//
//  BlindCareApp.swift
//  BlindCare
//
//  Created by Nipun Fernando on 06/11/2025.
//

import SwiftUI
import UserNotifications



@main
struct BlindCareApp: App {
    
    // Instantiate the single source of truth using @State
    @State private var userData = UserData()
    
    init() {
        requestNotificationPermission()
    }

    
    var body: some Scene {
        WindowGroup {
            ContentView()
                .environment(userData)
                .task {
                    await userData.loadInitialSnapshot()
                    userData.startMonitoringGPS()
                }
        }
    }
    
    private func requestNotificationPermission() {
        UNUserNotificationCenter.current().requestAuthorization(options: [.alert, .sound, .badge]) { granted, error in
            if let error = error {
                print("Notification permission error: \(error)")
            }
            print("Notifications allowed: \(granted)")
        }
    }

}
