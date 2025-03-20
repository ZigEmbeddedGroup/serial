# Zig Serial Port Library

Library for configuring and listing serial ports.

## Features

- Basic serial port configuration
  - Baud Rate
  - Parity (none, even, odd, mark, space)
  - Stop Bits (one, two)
  - Handshake (none, hardware, software)
  - Byte Size (5, 6, 7, 8)
- Flush serial port send/receive buffers
- List available serial ports
- API: supports Windows, Linux and Mac

## Example

```zig
// Port configuration.
// Serial ports are just files, \\.\COM1 for COM1 on Windows:
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

## Usage

### Library integration

Integrate the library in your project via the Zig package manager:

- add `serial` to your `.zig.zon` file by providing the URL to the archive of a tag or specific commit of the library
- to update the hash, run `zig fetch --save [URL/to/tag/or/commit.tar.gz]`

### Running tests

The `build.zig` file contains a test step that can be called with `zig build test`. Note that this requires a serial port to be available on the system;

- Linux: `/dev/ttyUSB0`
- Mac: `/dev/cu.usbmodem101`
- Windows: `COM3`

### Building the examples

You can build the examples from the `./examples` directory by calling `zig build examples`. Binaries will be generated in `./zig-out/bin` by default.

- Note that the `list_port_info` example currently only works on Windows

### Building the documentation

You can generate the documentation by running `zig build docs`.
After that you can browse it by:

 1. starting the web server. For example, by running `python -m http.server 8000 zig-out/docs`
 2. reading the docs from your browser at `http://127.0.0.1:8000`

