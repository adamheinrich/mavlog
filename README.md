# mavlog

Mavlog is a simple [MavLink][1] logger. It is designed to log messages sent by
the [PX4Flow][2] sensor over UART or USB.

## Build instructions

The code depends on MavLink's [c_library_v1][3] repository which is included as
a submodule. Make sure it has been initialized:

```
git submodule init
```

Then simply run `make` in the `src` directory.

## Usage

To log messages from a sensor connected to `/dev/ttyACM0` into file `log.csv`,
run:

```
mavlog /dev/ttyACM0 log.csv
```

Run `mavlog -v <port> <filename>` if you like verbose output.

## Output file format

The logger logs three message types into a CSV file separated by semicolon. Rows
with less than 14 numbers are padded by zeros. The `timestamp` value is a system
time in microseconds.

[OPTICAL_FLOW][4]:
```
timestamp;100;time_usec;flow_comp_m_x;flow_comp_m_y;ground_distance;flow_x;flow_y;sensor_id;quality;0;0;0;0
```

[OPTICAL_FLOW_RAD][5]:
```
timestamp;106;time_usec;integration_time_us;integrated_x;integrated_y;integrated_xgyro;integrated_ygyro;integrated_zgyro;time_delta_distance_us;distance;temperature;sensor_id;quality
```

[DEBUG_VECT][6]:
```
timestamp;250;time_usec;x;y;z;0;0;0;0;0;0;0;0
```

## Sensor configuration

The PX4Flow module does not have the ability to store parameters in a
nonvolatile memory. It is therefore necessary to set desired parameters once the
module is powered on.

Mavlog relies on the default configuration except for the `USB_SEND_VIDEO`
parameter which is set to zero.

If you want to set any other [parameter][7], simply modify function
`configure_px4flow()` in `mavlog.c`.

## Known limitations

The baudrate is hard-coded to 115200 kbps. This works correctly for a PF4Flow
connected via USB (as the baudrate settings have no effect) and should also work
for a PX4Flow connected via UART.

If you want a different configuration, simply change relevant lines in
`serial.c`:

```
cfsetispeed(&options, B115200);
cfsetospeed(&options, B115200);
```

[1]: https://github.com/mavlink/mavlink
[2]: https://pixhawk.org/modules/px4flow
[3]: https://github.com/mavlink/c_library_v1
[4]: https://pixhawk.ethz.ch/mavlink/#OPTICAL_FLOW
[5]: https://pixhawk.ethz.ch/mavlink/#OPTICAL_FLOW_RAD
[6]: https://pixhawk.ethz.ch/mavlink/#DEBUG_VECT
[7]: https://pixhawk.org/dev/px4flow
