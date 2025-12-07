module.exports = async function (context, req) {
  // The Table input binding can return a single entity or an array depending on config.
  const raw = context.bindings.latestEntity;
  const e = Array.isArray(raw) ? raw[0] : raw;

  context.log("LATEST RAW:", JSON.stringify(raw));  // shows up in Logs

  if (!e) {
    context.res = { status: 404, body: { error: "No data stored yet" } };
    return;
  }

  // Return everything we have so we can see exact property names/types.
  context.res = {
    status: 200,
    body: {
      // Azure Table system keys
      PartitionKey: e.PartitionKey ?? null,
      RowKey:       e.RowKey ?? null,
      Timestamp:    e.Timestamp ?? null,
      // Your custom fields as stored
      device_id:    e.device_id ?? null,
      received_at:  e.received_at ?? null,
      alert:        e.alert ?? null,
      lat_deg:      e.lat_deg ?? null,
      lon_deg:      e.lon_deg ?? null,
      // Also include the raw object for full visibility (remove later)
      _raw: e
    }
  };
};
