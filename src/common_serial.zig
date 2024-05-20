const std = @import("std");
const linux_serial = @import("linux/serial.zig");
const darwin_serial = @import("darwin/serial.zig");
const windows_serial = @import("windows/serial.zig");
const native_os = @import("builtin").os.tag;

pub const Iterator = switch (native_os) {
    .windows => @import("windows/Iterator.zig"),
    .linux => @import("linux/Iterator.zig"),
    .macos => @import("darwin/Iterator.zig"),
    else => @compileError("OS is not supported for port iteration"),
};

pub const InformationIterator = switch (native_os) {
    .windows => @import("windows/InformationIterator.zig"),
    .linux, .macos => @panic("'Port Information' not yet implemented for this OS"),
    else => @compileError("OS is not supported for information iteration"),
};

pub const Description = struct {
    file_name: []const u8,
    display_name: []const u8,
    driver: ?[]const u8,
};

pub const Information = struct {
    port_name: []const u8,
    system_location: []const u8,
    friendly_name: []const u8,
    description: []const u8,
    manufacturer: []const u8,
    serial_number: []const u8,
    // TODO: review whether to remove `hw_id`.
    // Is this useless/being used in a Windows-only way?
    hw_id: []const u8,
    vid: u16,
    pid: u16,
};

pub const Parity = enum {
    /// No parity bit is used
    none,
    /// Parity bit is `0` when an even number of bits is set in the data.
    even,
    /// Parity bit is `0` when an odd number of bits is set in the data.
    odd,
    /// Parity bit is always `1`
    mark,
    /// Parity bit is always `0`
    space,
};

pub const StopBits = enum {
    /// The length of the stop bit is 1 bit
    one,
    /// The length of the stop bit is 2 bits
    two,
};

pub const Handshake = enum {
    /// No handshake is used
    none,
    /// XON-XOFF software handshake is used.
    software,
    /// Hardware handshake with RTS/CTS is used.
    hardware,
};

pub const WordSize = enum {
    five,
    six,
    seven,
    eight,
};

pub const Config = struct {
    /// Symbol rate in bits/second. Not that these
    /// include also parity and stop bits.
    baud_rate: BaudRate = .B9600,

    /// Parity to verify transport integrity.
    parity: Parity = .none,

    /// Number of stop bits after the data
    stop_bits: StopBits = .one,

    /// Number of data bits per word.
    /// Allowed values are 5, 6, 7, 8
    word_size: WordSize = .eight,

    /// Defines the handshake protocol used.
    handshake: Handshake = .none,
};

pub const BaudRate = switch (native_os) {
    .windows => windows_serial.speed, // because windows has to be special
    else => std.c.speed_t, // std lib has nearly every os
};

pub const Buffers = enum { input, output, both };

pub const Pins = struct {
    rts: ?bool = null,
    dtr: ?bool = null,
};

pub fn configure(file: std.fs.File, config: Config) !void {
    const VTIME = 5;
    const VMIN = 6;
    const VSTART = 8;
    const VSTOP = 9;

    var settings = try std.posix.tcgetattr(file.handle);
    settings.ispeed = config.baud_rate;
    settings.ospeed = config.baud_rate;
    settings.lflag = .{};
    settings.oflag = .{};
    settings.iflag = .{
        .INPCK = (config.parity != .none),
        .IXON = (config.handshake == .software),
        .IXOFF = (config.handshake == .software),
    };
    settings.cflag = .{
        .CREAD = true,
        .PARENB = (config.parity != .none),
        .PARODD = (config.parity == .odd or config.parity == .mark),
        .CLOCAL = (config.handshake == .none),
        .CSTOPB = (config.stop_bits == .two),
        .CSIZE = switch (config.word_size) {
            .five => .CS5,
            .six => .CS6,
            .seven => .CS7,
            .eight => .CS8,
        },
    };
    switch (config.parity) {
        .mark => settings.cflag._ |= (1 << 14),
        .space => settings.cflag._ |= 1,
        else => {},
    }
    if (config.handshake == .hardware) settings.cflag._ |= (1 << 15);
    settings.cc[VMIN] = 1;
    settings.cc[VSTOP] = 0x13; // XOFF
    settings.cc[VSTART] = 0x11; // XON
    settings.cc[VTIME] = 0;
    // settings.cflag &= ~@as(os.tcflag_t, CBAUD);
    // settings.cflag |= baudmask;
    try std.posix.tcsetattr(file.handle, .NOW, settings);
}