const std = @import("std");
const common_serial = @import("../common_serial.zig");

pub fn flush(file: std.fs.File, buffers: common_serial.Buffers) !void {
    const mode = switch (buffers) {
        .input => 0,
        .output => 1,
        .both => 2,
    };
    const TCFLUSH: usize = 0x540B;
    if (std.os.linux.syscall3(
        .ioctl,
        @bitCast(@as(isize, file.handle)), // bitcast infers result type from arg1 // should
        TCFLUSH,
        mode,
    ) != 0) return error.flush;
}

pub fn changeControlPins(file: std.fs.File, pins: common_serial.Pins) !void {
    const TIOCM_RTS: c_int = 0x004;
    const TIOCM_DTR: c_int = 0x002;
    const TIOCMGET: u32 = 0x5415;
    const TIOCMSET: u32 = 0x5418;
    var flags: c_int = 0;

    if (std.os.linux.ioctl(
        file.handle,
        TIOCMGET,
        @intFromPtr(&flags),
    ) != 0) return error.control_pins;

    if (pins.dtr) |dtr| {
        if (dtr) flags |= TIOCM_DTR else flags &= ~TIOCM_DTR;
    }
    if (pins.rts) |rts| {
        if (rts) flags |= TIOCM_RTS else flags &= ~TIOCM_RTS;
    }

    if (std.os.linux.ioctl(
        file.handle,
        TIOCMSET,
        @intFromPtr(&flags),
    ) != 0) return error.control_pins;
}