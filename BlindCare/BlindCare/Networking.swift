//
//  Networking.swift
//  BlindCare
//
//  Created by Nipun Fernando on 12/11/2025.
//


import Foundation

// The interval for continuous polling in nanoseconds (5 seconds)
private let POLLING_INTERVAL: UInt64 = 5_000_000_000

// --- Data Model Refactoring ---
// Model matching the JSON keys from the Azure Function endpoint.
struct DeviceSnapshot: Codable {
    let deviceId: String
    let receivedAt: String
    let alert: Bool
    let latitude: Double
    let longitude: Double

    // Mapping Swift property names to Python/JSON keys
    enum CodingKeys: String, CodingKey {
        case deviceId = "device_id"
        case receivedAt = "received_at"
        case alert
        case latitude = "lat_deg"
        case longitude = "lon_deg"
    }
}

// GET link
let azureURLString = "https://smart-cane-function-avabctd3hvhwb0bw.westeurope-01.azurewebsites.net/api/get_latest?code=_jCGyMDXDv0XmmAtuAN1o38RAZ2LyNaumR2eFXZIkSMCAzFu53ZaXw=="

// POST link
let azureActionURLString = "https://smart-cane-function-avabctd3hvhwb0bw.westeurope-01.azurewebsites.net/api/send_downlink?code=dgOScDNNXS3zuyRg7CKZUrSKS7fQ27iF7cyszVhA7poCAzFuq0ZJVA=="


// -------------------------------------------------------------------
// MARK: - 1. REST API Functions
// -------------------------------------------------------------------

/**
 * Fetches the latest device snapshot (including location and alert status)
 * from the fixed Azure Function URL.
 */
func fetchLatestSnapshot() async -> DeviceSnapshot? {
    guard let url = URL(string: azureURLString) else {
        print("Error: Invalid Azure Function GET URL.")
        return nil
    }
    
    var request = URLRequest(url: url)
    request.httpMethod = "GET"
    
    do {
        let (data, response) = try await URLSession.shared.data(for: request)
        
        guard let httpResponse = response as? HTTPURLResponse, httpResponse.statusCode == 200 else {
            print("Error: Non-200 status code for GET request: \((response as? HTTPURLResponse)?.statusCode ?? 0)")
            return nil
        }
        
        let snapshot = try JSONDecoder().decode(DeviceSnapshot.self, from: data)
        print("Snapshot fetched successfully.")
        return snapshot
        
    } catch {
        print("Error fetching snapshot: \(error.localizedDescription)")
        return nil
    }
}

/**
 * Sends an acknowledgement payload ("01") to the server after a distress signal is accepted.
 *
 * - Parameter payload: The message to send, e.g., "01".
 * - Returns: True if the acknowledgment was successfully sent (200 status code).
 */
func sendAcknowledgement(payload: String) async -> Bool {
    guard let url = URL(string: azureActionURLString) else {
        print("Error: Invalid Azure Function POST URL.")
        return false
    }

    var request = URLRequest(url: url)
    request.httpMethod = "POST"
    request.setValue("application/json", forHTTPHeaderField: "Content-Type")

    // Match Python JSON structure
    let body: [String: Any] = [
        "device_id": "smart-cane",
        "payload_hex": payload,
        "fport": 1,
        "confirmed": false
    ]

    do {
        request.httpBody = try JSONSerialization.data(withJSONObject: body)

        let (_, response) = try await URLSession.shared.data(for: request)

        guard let httpResponse = response as? HTTPURLResponse,
              (200...299).contains(httpResponse.statusCode) else {
            print("Error: Non-2xx status code for POST request: \((response as? HTTPURLResponse)?.statusCode ?? 0)")
            return false
        }

        print("Payload '\(payload)' sent successfully.")
        return true

    } catch {
        print("Error sending payload: \(error.localizedDescription)")
        return false
    }
}


// -------------------------------------------------------------------
// MARK: - 2. Single Fetch Function (Poller Refactored)
// -------------------------------------------------------------------

/**
 * Starts a continuous background polling loop to fetch the device snapshot every 10 seconds.
 * - updateHandler: Closure to call on the MainActor when new data is successfully received.
 * - Returns: A Task<Void, Never> handle that can be cancelled to stop polling.
 */
func startGPSPolling(
    updateHandler: @escaping (DeviceSnapshot) -> Void
) -> Task<Void, Never> {
    
    let pollingTask = Task.detached {
        print("GPS Polling Task started. Fetching every \(await POLLING_INTERVAL / 1_000_000_000) seconds.")

        while !Task.isCancelled {
           
            if let snapshot = await fetchLatestSnapshot() {

                await MainActor.run {
                    updateHandler(snapshot)
                }
            } else {
                print("Polling fetch failed. Retrying...")
            }

            do {
                try await Task.sleep(nanoseconds: POLLING_INTERVAL)
            } catch {

                break
            }
        }
        print("GPS Polling Task stopped.")
    }
    
    return pollingTask
}
/**
 * Stops the GPS Polling Task
 */
func stopGPSPolling(task: Task<Void, Never>?) {
    task?.cancel()
    print("Requested cancellation of GPS Single Fetch Task.")
}
