const std = @import("std");
const zig_serial = @import("serial");

pub fn main() !u8 {
    const port_name = if (@import("builtin").os.tag == .windows) "\\\\.\\COM1" else "/dev/ttyUSB0";

    var serial = std.fs.cwd().openFile(port_name, .{ .mode = .read_write }) catch |err| switch (err) {
        error.FileNotFound => {
            std.debug.print("Invalid config: the serial port '{s}' does not exist.\n", .{port_name});
            return 1;
        },
        else => return err,
    };
    defer serial.close();

    try zig_serial.configureSerialPort(serial, zig_serial.SerialConfig{
        .baud_rate = 115200,
        .word_size = .eight,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });

    // NOTE: everything is written directly to the serial port so there is no
    // need to flush

    var writer = serial.writer(&.{});
    var reader = serial.reader(&.{});

    try writer.interface.writeAll("Hello, World!\r\n");

    while (true) {
        const b = try reader.interface.takeByte();
        try writer.interface.writeByte(b);
    }

    return 0;
}
