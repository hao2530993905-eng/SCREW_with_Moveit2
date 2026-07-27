# Screw motor feedback monitor

This tool reads the OpenArm/Damiao motor feedback used as the screw driver axis.
It prints:

- `position_rad`: motor position in rad
- `velocity_rad_s`: motor velocity in rad/s
- `measured_rpm`: velocity converted to rpm
- `torque_nm`: motor torque in N*m
- `mos_temperature_c`: MOS temperature
- `rotor_temperature_c`: rotor temperature

Build on the Ubuntu controller:

```bash
cd /home/sjh/Desktop/liuhao-main/apps/screw_driver
cmake -S /home/sjh/Desktop/liuhao-main/apps/screw_driver -B /home/sjh/Desktop/liuhao-main/apps/screw_driver/build -DUSE_OPENARM_CAN=ON
cmake --build /home/sjh/Desktop/liuhao-main/apps/screw_driver/build
```

Run with the default CAN and motor IDs:

```bash
/home/sjh/Desktop/liuhao-main/apps/screw_driver/build/screw_monitor --openarm --can can0 --send-id 0x06 --recv-id 0x16 --motor-type 1
```

Run for 10 seconds, sampling every 20 ms:

```bash
/home/sjh/Desktop/liuhao-main/apps/screw_driver/build/screw_monitor --openarm --duration-sec 10 --period-ms 20
```

Command a slow velocity while monitoring feedback:

```bash
/home/sjh/Desktop/liuhao-main/apps/screw_driver/build/screw_monitor --openarm --speed-rpm 60 --duration-sec 5
```

Use `--no-can-fd` if the CAN bus is configured as classic CAN instead of CAN-FD.
