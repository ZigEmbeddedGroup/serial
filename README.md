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
- Listing available serial ports

## Example

```zig
const serial = @import("serial");

var iterator = serial.iterator();

var sp = serial.init("COM3", .{
    .baud_rate = .B19200,
    .word_size = .eight,
    .parity = .none,
    .stop_bits = .one,
    .handshake = .none,
});
defer sp.close();

serial.flush(sp, .both);
serial.changeControlPins(sp, .{ .rts = true, .dtr = false });
```
