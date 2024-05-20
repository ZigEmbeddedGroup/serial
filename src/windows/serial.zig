const std = @import("std");
const common_serial = @import("../common_serial.zig");

pub fn flush(file: std.fs.File, buffers: common_serial.Buffers) !void {
    const flags: u32 = switch (buffers) {
        .input => 0x0008,
        .output => 0x0004,
        .both => 0x0004 | 0x0008,
    };
    if (PurgeComm(file.handle, flags) == 0) return error.flush;
}

extern "kernel32" fn PurgeComm(
    hFile: std.os.windows.HANDLE,
    dwFlags: std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;

pub fn changeControlPins(file: std.fs.File, pins: common_serial.Pins) !void {
    const SETRTS = 3;
    const CLRRTS = 4;
    const SETDTR = 5;
    const CLRDTR = 6;

    if (pins.dtr) |dtr| {
        if (EscapeCommFunction(file.handle, if (dtr) SETDTR else CLRDTR) == 0)
            return error.control_pins_dtr;
    }
    if (pins.rts) |rts| {
        if (EscapeCommFunction(file.handle, if (rts) SETRTS else CLRRTS) == 0)
            return error.control_pins_rts;
    }
}

extern "kernel32" fn EscapeCommFunction(
    hFile: std.os.windows.HANDLE,
    dwFunc: std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;

pub fn configure(file: std.fs.File, config: common_serial.Config) !void {
    var dcb = std.mem.zeroes(DCB);
    dcb.DCBlength = @sizeOf(DCB);
    if (GetCommState(file.handle, &dcb) == 0) return error.configure;
    dcb.BaudRate = @intFromEnum(config.baud_rate);
    const dcb_flags = DCBFlags{
        .fBinary = 1,
        .fParity = @intFromBool(config.parity != .none),
        .fOutxCtsFlow = @intFromBool(config.handshake == .hardware),
        .fOutX = @intFromBool(config.handshake == .software),
        .fInX = @intFromBool(config.handshake == .software),
        .fRtsControl = @intFromBool(config.handshake == .hardware),
    };
    dcb.flags = dcb_flags.toNumeric();
    dcb.wReserved = 0;
    dcb.wReserved1 = 0;
    dcb.XonChar = 0x11;
    dcb.XoffChar = 0x13;
    dcb.StopBits = switch (config.stop_bits) {
        .one => 0,
        .two => 2,
    };
    dcb.ByteSize = switch (config.word_size) {
        .five => 5,
        .six => 6,
        .seven => 7,
        .eight => 8,
    };
    dcb.Parity = switch (config.parity) {
        .none => 0,
        .odd => 1,
        .even => 2,
        .mark => 3,
        .space => 4,
    };
    if (SetCommState(file.handle, &dcb) == 0) return error.configure;
}

const DCB = extern struct {
    DCBlength: std.os.windows.DWORD,
    BaudRate: std.os.windows.DWORD,
    flags: u32,
    wReserved: std.os.windows.WORD,
    XonLim: std.os.windows.WORD,
    XoffLim: std.os.windows.WORD,
    ByteSize: std.os.windows.BYTE,
    Parity: std.os.windows.BYTE,
    StopBits: std.os.windows.BYTE,
    XonChar: u8,
    XoffChar: u8,
    ErrorChar: u8,
    EofChar: u8,
    EvtChar: u8,
    wReserved1: std.os.windows.WORD,
};

const DCBFlags = packed struct {
    fBinary: u1 = 1, // u1
    fParity: u1 = 0, // u1
    fOutxCtsFlow: u1 = 0, // u1
    fOutxDsrFlow: u1 = 0, // u1
    fDtrControl: u2 = 0, // u2
    fDsrSensitivity: u1 = 0, // u1
    fTXContinueOnXoff: u1 = 0, // u1
    fOutX: u1 = 0, // u1
    fInX: u1 = 0, // u1
    fErrorChar: u1 = 0, // u1
    fNull: u1 = 0, // u1
    fRtsControl: u2 = 0, // u2
    fAbortOnError: u1 = 0, // u1
    fDummy2: u17 = 0, // u17

    pub fn toNumeric(self: DCBFlags) u32 {
        var value: u32 = 0;
        value += @as(u32, self.fBinary) << 0; // u1
        value += @as(u32, self.fParity) << 1; // u1
        value += @as(u32, self.fOutxCtsFlow) << 2; // u1
        value += @as(u32, self.fOutxDsrFlow) << 3; // u1
        value += @as(u32, self.fDtrControl) << 4; // u2
        value += @as(u32, self.fDsrSensitivity) << 6; // u1
        value += @as(u32, self.fTXContinueOnXoff) << 7; // u1
        value += @as(u32, self.fOutX) << 8; // u1
        value += @as(u32, self.fInX) << 9; // u1
        value += @as(u32, self.fErrorChar) << 10; // u1
        value += @as(u32, self.fNull) << 11; // u1
        value += @as(u32, self.fRtsControl) << 12; // u2
        value += @as(u32, self.fAbortOnError) << 14; // u1
        value += @as(u32, self.fDummy2) << 15; // u17
        return value;
    }
};

extern "kernel32" fn GetCommState(
    hFile: std.os.windows.HANDLE,
    lpDCB: *DCB,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;

extern "kernel32" fn SetCommState(
    hFile: std.os.windows.HANDLE,
    lpDCB: *DCB,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;

pub const speed = enum(u32) {
    B0 = 0,
    B50 = 50,
    B75 = 75,
    B110 = 110,
    B134 = 134,
    B150 = 150,
    B200 = 200,
    B300 = 300,
    B600 = 600,
    B1200 = 1200,
    B1800 = 1800,
    B2400 = 2400,
    B4800 = 4800,
    B7200 = 7200,
    B9600 = 9600,
    B14400 = 14400,
    B19200 = 19200,
    B28800 = 28800,
    B31250 = 31250,
    B38400 = 38400,
    B57600 = 57600,
    B76800 = 76800,
    B115200 = 115200,
    B153600 = 153600,
    B230400 = 230400,
    B307200 = 307200,
    B460800 = 460800,
    B500000 = 500000,
    B576000 = 576000,
    B921600 = 921600,
    B1000000 = 1000000,
    B1152000 = 1152000,
    B1500000 = 1500000,
    B2000000 = 2000000,
    B2500000 = 2500000,
    B3000000 = 3000000,
    B3500000 = 3500000,
    B4000000 = 4000000,
};