// Azure Functions v4 (Node 18+)
// Sends a LoRaWAN downlink via TTN v3 /as/applications/.../down/push
// Now: sends the requested payload first (e.g. "01"), then "00" after 2 seconds.

const REGION = process.env.TTN_REGION_BASE;            // e.g., "eu1.cloud.thethings.network"
const APP_ID = process.env.TTN_APP_ID;
const API_KEY = process.env.TTN_API_KEY;
const DEFAULT_DEV = process.env.TTN_DEVICE_ID_DEFAULT || "smart-cane";

// Map friendly "message" values -> payload bytes (hex)
const MESSAGE_MAP = {
  received: "01"  // hex 0x01
  // you can also add e.g. cleared: "00" if you want a named message for that
};

// Helpers
function hexToBase64(hex) {
  const clean = hex.replace(/\s+/g, "");
  const bytes = Uint8Array.from(clean.match(/.{1,2}/g).map((b) => parseInt(b, 16)));
  return Buffer.from(bytes).toString("base64");
}

// Simple delay helper
function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

// Helper to send ONE downlink to TTN
async function sendDownlinkToTTN({ deviceId, frmB64, fport, confirmed, context }) {
  const url = `https://${REGION}/api/v3/as/applications/${APP_ID}/devices/${deviceId}/down/push`;

  const resp = await fetch(url, {
    method: "POST",
    headers: {
      "Authorization": `Bearer ${API_KEY}`,
      "Content-Type": "application/json"
    },
    body: JSON.stringify({
      downlinks: [
        {
          frm_payload: frmB64, // base64
          f_port: fport,
          confirmed
        }
      ]
    })
  });

  if (!resp.ok) {
    const text = await resp.text();
    context.log.error("TTN downlink error:", resp.status, text);
    return { ok: false, status: resp.status, error: text.slice(0, 300) };
  }

  const out = await resp.json().catch(() => ({}));
  return { ok: true, status: 200, ttn: out };
}

module.exports = async function (context, req) {
  try {
    // 1) Parse input
    const body = req.body || {};
    const deviceId  = body.device_id || DEFAULT_DEV;
    const fport     = Number(body.fport ?? 1);
    const confirmed = Boolean(body.confirmed ?? false);

    // 2) Figure out payload (base64) for the FIRST downlink
    // Priority: payload_raw_base64 > payload_hex > message
    let frmB64 = body.payload_raw_base64;
    let rawHexUsed = null; // to know if it was "01"

    if (!frmB64 && body.payload_hex) {
      frmB64 = hexToBase64(body.payload_hex);
      rawHexUsed = body.payload_hex;
    }

    if (!frmB64 && body.message) {
      const hex = MESSAGE_MAP[body.message];
      if (!hex) {
        context.res = { status: 400, body: { error: `Unknown message '${body.message}'` } };
        return;
      }
      frmB64 = hexToBase64(hex);
      rawHexUsed = hex;
    }

    if (!frmB64) {
      context.res = { status: 400, body: { error: "Provide payload_raw_base64 OR payload_hex OR message" } };
      return;
    }

    // 3) Send FIRST downlink (e.g. '01')
    const firstResult = await sendDownlinkToTTN({
      deviceId,
      frmB64,
      fport,
      confirmed,
      context
    });

    // If the first one fails, you might want to stop here
    if (!firstResult.ok) {
      context.res = { status: 200, body: { ok: false, step: "first_downlink", ...firstResult } };
      return;
    }

    // 4) Wait 3 seconds and send SECOND downlink '00'
    // You can decide whether to ALWAYS send '00' or only when the first was '01'.
    // Here: send '00' only if the first payload was 0x01 (or message 'received').
    let secondResult = null;
    const normalizedHex = rawHexUsed ? rawHexUsed.replace(/\s+/g, "").toLowerCase() : null;
    if (normalizedHex === "01") {
      await delay(6500); //  seconds

      const frmB64_zero = hexToBase64("00");
      secondResult = await sendDownlinkToTTN({
        deviceId,
        frmB64: frmB64_zero,
        fport,
        confirmed,
        context
      });
    }

    // 5) Return final result
    context.res = {
      status: 200,
      body: {
        ok: firstResult.ok && (!secondResult || secondResult.ok),
        device_id: deviceId,
        fport,
        confirmed,
        first_downlink: firstResult,
        second_downlink: secondResult || null
      }
    };

  } catch (err) {
    context.log.error("send_downlink crash:", err);
    context.res = { status: 200, body: { ok: false, error: String(err) } };
  }
};
