# Finding out how to control Loona

The mapper needs four things from the robot (`pose`, `rotate`, `forward`,
`read_range`). Loona's app connects over Wi-Fi, but that does not tell us
whether it uses a **local** protocol or a **vendor cloud**. Work through these
steps on your own network and robot, and note what you find. All of this is
for a device you own.

## 1. Which kind of Wi-Fi link is it?

* **Direct (robot makes its own hotspot, phone joins it):** local control is
  very likely. The robot's IP is usually the gateway of that hotspot.
* **Same home network (robot and phone both join your router):** could be
  local, could be cloud-relayed. Find the robot in your router's device list.

Quick test: with the robot on, put the phone in airplane mode, turn Wi-Fi back
on and join only the robot's network (or block the robot's internet access at
the router). If the app still drives it, there is a local protocol.

## 2. Probe it

Run on a computer on the same network (not in the cloud session):

```bash
loona-map discover 192.168.1.50        # the robot's IP
loona-map discover 192.168.1.0/24      # or sweep the subnet
```

Open TCP ports with HTTP banners suggest a web API or WebSocket. Nothing open
suggests UDP, Bluetooth, or cloud-only. `discover` only covers TCP.

## 3. Watch what the app says

* **Packet capture:** run Wireshark/tcpdump on a machine that sees the traffic
  (join the robot's hotspot from a laptop and share it onward, or use a
  router that can mirror/capture). Filter on the robot's IP and drive it from
  the app. Look for small repeating packets on movement, and for periodic
  sensor frames.
* **HTTP(S) proxy:** mitmproxy on the computer, phone's Wi-Fi proxy pointed at
  it. Works for plain HTTP and for apps that trust user-installed
  certificates. Apps that pin certificates will refuse, and that is a sign to
  fall back to packet capture.
* **Unencrypted vs encrypted:** if the payloads are readable JSON/binary,
  replaying them is straightforward. If they are TLS to a vendor domain, the
  control is cloud-mediated and this project cannot drive the robot without
  the vendor's cooperation or an official API.

## 4. What to bring back

Even rough notes are enough to write the adapter:

* the protocol (HTTP / WebSocket / UDP / MQTT / BLE) and port
* one captured request that makes Loona drive forward and one that turns it
* whatever message carries the distance-sensor reading, if any
* whether the robot reports odometry or heading

Without a readable distance sensor there is nothing to map from. In that case
the fallback is vision from the camera stream (a different, larger project).
