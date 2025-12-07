module.exports = async function (context, req) {
  try {
    const body = req.body || {};
    const devId = body?.end_device_ids?.device_id;
    const dp    = body?.uplink_message?.decoded_payload;

    const valid = devId && dp &&
      typeof dp.alert   !== "undefined" &&
      typeof dp.lat_deg !== "undefined" &&
      typeof dp.lon_deg !== "undefined";

    if (!valid) {
      context.res = { status: 200, body: { ok: true, ignored: true } };
      return;
    }

    const ts = body?.received_at || new Date().toISOString();

    const snapshot = {
      device_id  : devId,
      received_at: ts,
      alert      : !!dp.alert,
      lat_deg    : Number(dp.lat_deg),
      lon_deg    : Number(dp.lon_deg)
    };

    // 1) Append to history (always a new row)
    context.bindings.outHistory = {
      PartitionKey: devId,
      RowKey      : ts,
      ...snapshot
    };

    // 2) Upsert latest (InsertOrReplace via etag:"*")
    context.bindings.outLatest = {
      PartitionKey: devId,
      RowKey      : "latest",
      etag        : "*",           // <-- THIS makes it an upsert
      ...snapshot
    };

    context.log(`Stored uplink from ${devId} @ ${ts}`);
    context.res = { status: 200, body: { stored: true, device_id: devId, at: ts } };
  } catch (err) {
    context.log.error("ttn_uplink error:", err);
    // keep TTN happy while you debug
    context.res = { status: 200, body: { ok: false, error: String(err) } };
  }
};
