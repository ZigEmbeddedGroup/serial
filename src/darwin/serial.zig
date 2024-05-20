const std = @import("std");
const c = @cImport(@cInclude("termios.h"));
const common_serial = @import("../common_serial.zig");

pub fn flush(file: std.fs.File, buffers: common_serial.Buffers) !void {
    const mode = switch (buffers) {
        .input => c.TCIFLUSH,
        .output => c.TCOFLUSH,
        .both => c.TCIOFLUSH,
    };
    if (c.tcflush(file.handle, @as(c_int, @intCast(mode))) != 0) return error.flush;
}