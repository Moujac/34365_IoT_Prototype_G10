// ContentView.swift
// BlindCare
//
// Created by Nipun Fernando on 06/11/2025.

import SwiftUI
import MapKit
import CoreLocation   // For CLGeocoder
import UIKit          // For UIPasteboard

// Requires access to UserData.swift

struct ContentView: View {

    // Access the shared UserData instance injected by BlindCareApp
    @Environment(UserData.self) private var userData

    // Track network operation
    @State private var isAcknowledging: Bool = false

    // Derived coordinate for the map marker
    private var locationCoordinate: CLLocationCoordinate2D {
        CLLocationCoordinate2D(latitude: userData.currentLatitude,
                               longitude: userData.currentLongitude)
    }

    // Map camera state
    @State private var cameraPosition: MapCameraPosition = .automatic

    // Display for the polling interval
    private var pollingIntervalDisplay: String {
        userData.isPollingActive ? "Polling Active (Fetching every 10s)" : "Polling Inactive"
    }

    var body: some View {
        VStack(spacing: 20) {

            Text("Smart Cane Monitor")
                .font(.largeTitle)
                .fontWeight(.bold)
                .padding(.bottom, 10)

            Text("Current Location")
                .font(.headline)

            // --- Map View ---
            Map(position: $cameraPosition) {

                if userData.currentLatitude != 0.0 || userData.currentLongitude != 0.0 {
                    Marker("Device Location", coordinate: locationCoordinate)
                        .tint(userData.currentAlertStatus ? .red : .green)
                }
            }
            .frame(height: 300)
            .cornerRadius(12)
            .shadow(radius: 5)
            .onChange(of: userData.currentLatitude, initial: false) { _, newLatitude in
                let newCoordinate = CLLocationCoordinate2D(
                    latitude: newLatitude,
                    longitude: userData.currentLongitude
                )
                updateMapCamera(newLatitude: newCoordinate.latitude,
                                newLongitude: newCoordinate.longitude)
                Task { await reverseGeocode(coordinate: newCoordinate) }
            }
            .onChange(of: userData.currentLongitude, initial: false) { _, newLongitude in
                let newCoordinate = CLLocationCoordinate2D(
                    latitude: userData.currentLatitude,
                    longitude: newLongitude
                )
                updateMapCamera(newLatitude: newCoordinate.latitude,
                                newLongitude: newCoordinate.longitude)
                Task { await reverseGeocode(coordinate: newCoordinate) }
            }
            .onAppear {
                updateMapCamera(newLatitude: userData.currentLatitude,
                                newLongitude: userData.currentLongitude,
                                animated: false)
                Task { await reverseGeocode(coordinate: locationCoordinate) }
            }

            // --- Address & Copy Button ---
            VStack(alignment: .leading, spacing: 8) {

                HStack(alignment: .top) {
                    Text("Address:")
                        .fontWeight(.semibold)

                    Text(userData.currentAddress)
                        .lineLimit(3)
                        .fixedSize(horizontal: false, vertical: true)
                        .frame(minWidth: 1, alignment: .leading)
                }
                .frame(maxWidth: .infinity, alignment: .leading)

                Button {
                    UIPasteboard.general.string = userData.currentAddress
                } label: {
                    Label("Copy Address", systemImage: "doc.on.clipboard")
                }
                .buttonStyle(.bordered)
                .tint(.blue)
                .disabled(
                    userData.currentAddress == "Finding address..." ||
                    userData.currentAddress == "Address not found." ||
                    userData.currentAddress == "No data..." ||
                    userData.currentAddress.isEmpty
                )
            }
            .padding(.leading, 0)

            Divider()

            // --- Device Status ---
            HStack(alignment: .top, spacing: 20) {
                VStack(alignment: .leading, spacing: 10) {

                    Text("Device ID: \(userData.deviceId)")
                        .font(.title2)
                        .fontWeight(.medium)

                    HStack {
                        Text("Alert Status:")
                            .font(.system(size: 16))
                            .fontWeight(.semibold)

                        Text(userData.currentAlertStatus
                            ? "DISTRESS SIGNAL RECEIVED"
                            : "Normal"
                        )
                        .foregroundColor(userData.currentAlertStatus ? .white : .primary)
                        .font(.system(size: 15))
                    }
                    .padding(7)
                    .background(userData.currentAlertStatus ? Color.red : Color(.systemGray5))
                    .cornerRadius(8)

                    Text("Last Updated: \(userData.lastUpdated)")
                        .font(.footnote)
                        .foregroundColor(.secondary)
                }
                Spacer()
            }
            .padding(.horizontal, 5)

            Divider()

            // --- Controls ---
            VStack(spacing: 15) {

                if isAcknowledging {
                    ProgressView("Sending acknowledgement...")
                        .foregroundColor(.red)
                        .fontWeight(.semibold)
                } else {
                    /*Text(pollingIntervalDisplay)
                        .foregroundColor(userData.isPollingActive ? .green : .red)
                        .fontWeight(.semibold)*/
                }

                // Acknowledge button
                Button {
                    isAcknowledging = true
                    Task {
                        _ = await userData.sendDistressAcknowledgement()
                        isAcknowledging = false
                    }
                } label: {
                    Label("Accept Distress Signal",
                          systemImage: "antenna.radiowaves.left.and.right")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .tint(.green)
                .disabled(!userData.isPollingActive ||
                           !userData.currentAlertStatus ||
                            isAcknowledging)

                // Resolve alert locally
                Button {
                    userData.resolveDistressSignalLocally()
                } label: {
                    Label("Distress signal resolved",
                          systemImage: "checkmark.shield.fill")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .tint(.orange)
            }
            .padding(.horizontal)

            Spacer()
        }
        .padding()
    }

    // MARK: - Reverse Geocode
    @MainActor
    private func reverseGeocode(coordinate: CLLocationCoordinate2D) async {

        guard coordinate.latitude != 0.0 || coordinate.longitude != 0.0 else {
            if userData.isPollingActive {
                userData.currentAddress = "Fetching latest location..."
            }
            return
        }

        userData.currentAddress = "Finding address..."

        let location = CLLocation(latitude: coordinate.latitude,
                                  longitude: coordinate.longitude)

        guard let request = MKReverseGeocodingRequest(location: location) else {
            userData.currentAddress = "Unable to create geocode request."
            return
        }

        do {
            let mapItems = try await request.mapItems

            guard let item = mapItems.first else {
                userData.currentAddress = "Address not found."
                return
            }

            if let reps = item.addressRepresentations {

                if let full = reps.fullAddress(includingRegion: true, singleLine: true),
                   !full.isEmpty {
                    userData.currentAddress = full
                    return
                }

                if let city = reps.cityWithContext(.full), !city.isEmpty {
                    userData.currentAddress = city
                    return
                }

                if let city = reps.cityName, !city.isEmpty {
                    userData.currentAddress = city
                    return
                }

                if let region = reps.regionName, !region.isEmpty {
                    userData.currentAddress = region
                    return
                }
            }

            userData.currentAddress = "Address not found."

        } catch {
            userData.currentAddress = "Address error: \(error.localizedDescription)"
        }
    }

    // MARK: - Update Camera
    private func updateMapCamera(newLatitude: Double,
                                 newLongitude: Double,
                                 animated: Bool = false) {

        guard newLatitude != 0.0 || newLongitude != 0.0 else {
            cameraPosition = .automatic
            return
        }

        let newCoordinate = CLLocationCoordinate2D(latitude: newLatitude,
                                                   longitude: newLongitude)

        let region = MKCoordinateRegion(
            center: newCoordinate,
            span: MKCoordinateSpan(latitudeDelta: 0.005,
                                   longitudeDelta: 0.005)
        )

        withAnimation(.easeIn(duration: 0.5)) {
            cameraPosition = .region(region)
        }
    }
}


#Preview {
    // --- Mock UserData ---
    let mockUserData = UserData()
    mockUserData.deviceId = "ID-24"
    mockUserData.currentLatitude = 51.5074
    mockUserData.currentLongitude = -0.1278
    mockUserData.currentAddress = "DTU Lyngby Campus, Denmark"
    mockUserData.currentAlertStatus = true
    mockUserData.isPollingActive = true
    mockUserData.lastUpdated = "06/11/2025 14:32"

    return ContentView()
        .environment(mockUserData)
}
