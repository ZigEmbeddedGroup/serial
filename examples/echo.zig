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
        .baud_rate = 115_200,
        .word_size = .eight,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });

    // NOTE: everything is written directly to the serial port so there is no
    // need to flush (because there is no buffering).
    var writer = serial.writer(&.{});

    var r_buf: [128]u8 = undefined;
    var reader = serial.reader(&r_buf);

    try writer.interface.writeAll("Hello, World!\r\n");

    while (true) {
        const b = try reader.interface.takeByte();
        try writer.interface.writeByte(b);
    }

    return 0;
}
