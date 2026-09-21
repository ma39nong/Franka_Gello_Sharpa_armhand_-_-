# Orbbec Gemini 435Le

Verified hardware: Gemini 435Le `CP4N651006F`, camera `192.168.1.123:8090`, host camera address `192.168.1.53/24`. (The unit and/or its address has changed at least twice since this doc was first written — earlier revisions recorded `CP4E46300048`/`192.168.1.10` and `CP4N5630008Z`. Re-verify the serial with the SDK log or Viewer discovery rather than trusting any single doc.)

## Network

The camera is Ethernet/PoE, so it does not appear in `lsusb` or `/dev/video*`. Discovery can work across a misconfigured host while streams still fail; the host needs an address on `192.168.1.0/24`.

The current `Wired connection 1` profile carries both `172.16.0.6/24` for FR3 and `192.168.1.53/24` for the camera. Verify without changing the live interface:

```bash
nmcli -g ipv4.addresses connection show 'Wired connection 1'
ip -4 -br address show dev enp6s0
nc -vz -w 3 192.168.1.123 8090
```

If the persistent camera address is missing, add it only while FCI is stopped:

```bash
nmcli connection modify 'Wired connection 1' ipv4.addresses '172.16.0.6/24,192.168.1.53/24' ipv4.method manual
nmcli device modify enp6s0 +ipv4.addresses 192.168.1.53/24
```

Rollback:

```bash
nmcli connection modify 'Wired connection 1' ipv4.addresses '172.16.0.6/24'
nmcli device modify enp6s0 -ipv4.addresses 192.168.1.53/24
```

Never bounce or reconfigure `enp6s0` during an active FCI session.

## ROS

```bash
cd /home/descfly/llx/gello_upper_body_teleop/docker
docker compose up -d orbbec
docker compose logs -f orbbec
docker compose exec orbbec timeout 15 ros2 topic hz /camera/color/image_raw
docker compose exec orbbec timeout 15 ros2 topic hz /camera/depth/image_raw
```

Expected topics are `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/depth/image_raw`, and `/camera/depth/camera_info`. The head-camera collection profile is 640×400 at 20 FPS; Python `ros2 topic hz` can under-report while deserializing images, so `/camera/device_status` is the authoritative source counter.

The image pins the SDK v2 ROS wrapper and applies `docker/patches/orbbec_ros2_skip_uvc_for_network.patch`; removing that patch causes the Ethernet camera to fail on an irrelevant USB/UVC initialization.

## Viewer

Stop the ROS camera service first, then run:

```bash
cd /home/descfly/llx/gello_upper_body_teleop
./ops/run/start_orbbec_viewer.sh
```

Use the SDK v2 viewer selected by the script, not the old SDK v1 download. Only one Viewer or ROS client may own the camera.

Harvest three-camera collection (`/cam{0,1,2}/color/image_raw`) is a separate
bringup. Do not run it together with this compose `orbbec` service or
OrbbecViewer; `./ops/run/start_recording.sh` refuses those conflicts. See
[data_collection/README.md](../data_collection/README.md).


## FCI isolation

RGB-D traffic and the 1 kHz Franka sessions currently share `enp6s0`. Basic streaming and ping checks passed, but they do not prove worst-case FCI timing. Production data collection should put `192.168.1.53/24` on a dedicated Gigabit NIC and confirm `ip route get 192.168.1.123` selects it.
