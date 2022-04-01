# Zig Serial Port Helper

Helper library for configuring and listing serial ports.

## Features

- Basic serial port configuration
  - Baud Rate
  - Parity (none, even, odd, mark, space)
  - Stop Bits (one, two)
  - Handshake (none, hardware, software)
  - Byte Size (5, 6, 7, 8)
- Flushing of the serial port receive buffers
- Supports windows and linux with the API
- Listing available serial ports (currently only on linux)

## Example

```zig
// Serial ports are just files, \\.\COM1 for COM1 on windows:
var serial = try std.fs.cwd().openFile("\\\\.\\COM1", .{ .mode = .read_write }) ;
defer serial.close();

try zig_serial.configureSerialPort(serial, zig_serial.SerialConfig{
    .baud_rate = 19200,
    .word_size = 8,
    .parity = .none,
    .stop_bits = .one,
    .handshake = .none,
});
```
