const std = @import("std");
const zig_serial = @import("serial");

pub fn main() !u8 {
    const port_name = if (@import("builtin").os.tag == .windows) "\\\\.\\COM1" else "/dev/ttyUSB0";

    var serial = std.fs.cwd().openFile(port_name, .{ .mode = .read_write }) catch |err| switch (err) {
        error.FileNotFound => {
            try std.io.getStdOut().writer().print("The serial port {s} does not exist.\n", .{port_name});
            return 1;
        },
        else => return err,
    };
    defer serial.close();

    try zig_serial.configureSerialPort(serial, zig_serial.SerialConfig{
        .baud_rate = 115200,
        .word_size = 8,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });

    try serial.writer().writeAll("Hello, World!\r\n");

    while (true) {
        var b = try serial.reader().readByte();
        try serial.writer().writeByte(b);
    }

    return 0;
}
