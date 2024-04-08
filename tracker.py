import pysurvive
import sys
from websockets.sync.client import connect
import time
import os

if (telemetry_url := os.getenv("GRAFANA_URL")) is None or (grafana_token := os.getenv("GRAFANA_SERVICE_ACCOUNT_TOKEN")) is None:
    logger.warning("Telemetry not configured. Skipping...")
    ws = None
else:
    ws = connect(f"ws://{telemetry_url}/api/live/push/go2", additional_headers={'Authorization': f"Bearer {grafana_token}"})


# telemetry_url = "ws://localhost:3000/api/live/push/go2"
# bearer = "glsa_s3aCwWwq8koE8pNjnfTsLpbRq72lk0h2_1e0c311a"
# ws = connect(telemetry_url, additional_headers={'Authorization': f'Bearer {bearer}'})


actx = pysurvive.SimpleContext(sys.argv)

for obj in actx.Objects():
    print(str(obj.Name(), 'utf-8'))

while actx.Running():
    updated = actx.NextUpdated()
    if updated:
        poseObj = updated.Pose()
        poseData = poseObj[0]
        poseTimestamp = poseObj[1]
        
        # Stream to Grafana
        # Format at https://docs.influxdata.com/influxdb/v2/reference/syntax/line-protocol/
        influx_time = int(time.time() * 1_000_000_000)     
        ground_truth_data = f"tracker track_x={poseData.Pos[0]},track_y={poseData.Pos[1]},track_z={poseData.Pos[2]} {influx_time}"

        ws.send(ground_truth_data)
        print("%s: T: %f P: % 9f,% 9f,% 9f R: % 9f,% 9f,% 9f,% 9f"%(str(updated.Name(), 'utf-8'), poseTimestamp, poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]))
        # Position(x, y, z), Quaternion(w, x, y, z)

