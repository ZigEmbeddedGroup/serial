const std = @import("std");
const common_serial = @import("common_serial.zig");
const windows_serial = @import("windows/serial.zig");
const linux_serial = @import("linux/serial.zig");
const darwin_serial = @import("darwin/serial.zig");
const native_os = @import("builtin").os.tag;

pub fn iterator() !common_serial.Iterator {
    return try common_serial.Iterator.init();
}

pub fn infoIterator() !common_serial.InformationIterator {
    return try common_serial.InformationIterator.init();
}

pub fn init(name: []const u8, config: common_serial.Config) !std.fs.File {
    const prefix = if (native_os == .windows) "\\\\.\\" else "/dev/";
    var path: []const u8 = undefined;
    if (std.mem.startsWith(u8, name, prefix)) {
        path = name;
    } else {
        var buffer: [std.fs.max_path_bytes]u8 = undefined;
        var fba = std.heap.FixedBufferAllocator.init(&buffer);
        path = try std.fs.path.join(
            fba.allocator(),
            &.{ prefix, name },
        );
    }
    const file = try std.fs.cwd().openFile(
        path,
        .{ .mode = .read_write },
    );
    errdefer file.close();
    try configure(file, config);
    return file;
}

/// Flushes the serial port `port`. If `input` is set, all pending data in
/// the receive buffer is flushed, if `output` is set all pending data in
/// the send buffer is flushed.
pub fn flush(file: std.fs.File, buffers: common_serial.Buffers) !void {
    switch (native_os) {
        .windows => try windows_serial.flush(file, buffers),
        .linux => try linux_serial.flush(file, buffers),
        .macos => try darwin_serial.flush(file, buffers),
        else => @compileError("unsupported OS, please implement!"),
    }
}

pub fn changeControlPins(file: std.fs.File, pins: common_serial.Pins) !void {
    switch (native_os) {
        .windows => try windows_serial.changeControlPins(file, pins),
        .linux => try linux_serial.changeControlPins(file, pins),
        .macos => @panic("Not implemented"),
        else => @compileError("changeControlPins not implemented for " ++ @tagName(native_os)),
    }
}

/// This function configures a serial port with the given config.
/// `port` is an already opened serial port, on windows these
/// are either called `\\.\COMxx\` or `COMx`, on unixes the serial
/// port is called `/dev/ttyXXX`.
pub fn configure(file: std.fs.File, config: common_serial.Config) !void {
    switch (native_os) {
        .windows => return windows_serial.configure(file, config),
        else => common_serial.configure(file, config),
    }
}

test "iterate ports" {
    var it = try iterator();
    while (try it.next()) |port| {
        std.debug.print("{s} (file: {s}, driver: {?s})\n", .{ port.display_name, port.file_name, port.driver });
    }
}

test "basic configuration test" {
    const cfg = common_serial.Config{
        .handshake = .none,
        .baud_rate = .B115200,
        .parity = .none,
        .word_size = .eight,
        .stop_bits = .one,
    };

    var tty: []const u8 = undefined;

    switch (native_os) {
        .windows => tty = "\\\\.\\COM3",
        .linux => tty = "/dev/ttyUSB0",
        .macos => tty = "/dev/cu.usbmodem101",
        else => unreachable,
    }

    var port = try std.fs.cwd().openFile(tty, .{ .mode = .read_write });
    defer port.close();

    try configure(port, cfg);
}

test "basic flush test" {
    var tty: []const u8 = undefined;
    // if any, these will likely exist on a machine
    switch (native_os) {
        .windows => tty = "\\\\.\\COM3",
        .linux => tty = "/dev/ttyUSB0",
        .macos => tty = "/dev/cu.usbmodem101",
        else => unreachable,
    }
    var port = try std.fs.cwd().openFile(tty, .{ .mode = .read_write });
    defer port.close();

    try flush(port, .both);
    try flush(port, .input);
    try flush(port, .output);
}

test "change control pins" {
    _ = changeControlPins;
}

test "init" {

    const tty = switch (native_os) {
        .windows => "\\\\.\\COM3",
        .linux => "/dev/ttyUSB0",
        .macos => "/dev/cu.usbmodem101",
        else => @compileError("unsupported os"),
    };

    var port = try init(tty, .{
        .baud_rate = .B9600,
        .handshake = .none,
        .parity = .none,
        .word_size = .eight,
        .stop_bits = .one,
    });
    defer port.close();
}
