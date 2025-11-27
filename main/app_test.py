import datetime
import requests
import time

# Get latest uplink data from Azure Function
AZURE_URL = "https://smart-cane-function-avabctd3hvhwb0bw.westeurope-01.azurewebsites.net/api/get_latest?code=_jCGyMDXDv0XmmAtuAN1o38RAZ2LyNaumR2eFXZIkSMCAzFu53ZaXw==" 

def fetch_latest():
    try:
        resp = requests.get(AZURE_URL, timeout=10)
        resp.raise_for_status()

        data = resp.json()

        return data

    except Exception as e:
        print("❌ Error:", e)
        print("Raw response:", resp.text if 'resp' in locals() else "No response")
        return None
    
# Send downlink command to Azure Function
FUNC_URL = "https://smart-cane-function-avabctd3hvhwb0bw.westeurope-01.azurewebsites.net/api/send_downlink?code=dgOScDNNXS3zuyRg7CKZUrSKS7fQ27iF7cyszVhA7poCAzFuq0ZJVA=="

def send_hex(hex_payload):
    body = {
        "device_id": "smart-cane",
        "payload_hex": hex_payload,  # e.g., "01" or "00"
        "fport": 1,
        "confirmed": False           # avoid retries
    }
    r = requests.post(FUNC_URL, json=body, timeout=10)
    #print(hex_payload, r.status_code, r.text)

# Varaibles to perform get reuest every 5 seconds
last_checked = 0

# Main loop to monitor uplinks and send downlinks
while True:
    # Check every 5 seconds
    current_time = int(datetime.datetime.now().timestamp())
    if current_time - last_checked > 5:
        last_checked = current_time

        data = fetch_latest() # fetch latest uplink data
        alert_status = data.get("alert") if data else None
        #print(f"Checked at {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}, Alert status: {alert_status}")
        if alert_status == True:
            print("Alert detected!")
            latitude = data.get("lat_deg")
            longitude = data.get("lon_deg")
            print(f"Location: lat {latitude}, lon {longitude}")

            alert_resolved = False
            while not alert_resolved:
                user_input = input("Enter 's' to acknowledge alert: ")
                if user_input.lower() == 's':
                    # Trigger buzzer - aknowledgement of alert
                    send_hex("01")
                    print("Buzzer triggered.")
                    alert_resolved = True
                    last_checked = int(datetime.datetime.now().timestamp())  # reset last checked to avoid immediate re-check