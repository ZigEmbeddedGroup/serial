const std = @import("std");

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

pub const SerialConfig = struct {
    /// Symbol rate in bits/second. Not that these
    /// include also parity and stop bits.
    baud_rate: usize,

    /// Parity to verify transport integrity.
    parity: Parity = .none,

    /// Number of stop bits after the data
    stop_bits: StopBits = .one,

    /// Number of data bits per word.
    /// Allowed values are 5, 6, 7, 8
    word_size: u4 = 8,

    /// Defines the handshake protocol used.
    handshake: Handshake = .none,
};

// from linux headers
const OPOST = 0o0000001;

const ISIG = 0o0000001; //     Enable signals.
const ICANON = 0o0000002; //     Canonical input (erase and kill processing).
const XCASE = 0o0000004;
const ECHO = 0o0000010; //    Enable echo.
const ECHOE = 0o0000020; //    Echo erase character as error-correcting backspace.
const ECHOK = 0o0000040; //    Echo KILL.
const ECHONL = 0o0000100; //    Echo NL.
const NOFLSH = 0o0000200; //    Disable flush after interrupt or quit.
const TOSTOP = 0o0000400; //    Send SIGTTOU for background output.
const IEXTEN = 0o0100000;

const CBAUD = 0o000000010017; //Baud speed mask (not in POSIX).
const CBAUDEX = 0o000000010000;

const CSIZE = 0o0000060;
const CS5 = 0o0000000;
const CS6 = 0o0000020;
const CS7 = 0o0000040;
const CS8 = 0o0000060;
const CSTOPB = 0o0000100;
const CREAD = 0o0000200;
const PARENB = 0o0000400;
const PARODD = 0o0001000;
const HUPCL = 0o0002000;
const CLOCAL = 0o0004000;
const CMSPAR = 0o010000000000;
const CRTSCTS = 0o020000000000;
const IGNBRK = 0o0000001;
const BRKINT = 0o0000002;
const IGNPAR = 0o0000004;
const PARMRK = 0o0000010;
const INPCK = 0o0000020;
const ISTRIP = 0o0000040;
const INLCR = 0o0000100;
const IGNCR = 0o0000200;
const ICRNL = 0o0000400;
const IUCLC = 0o0001000;
const IXON = 0o0002000;
const IXANY = 0o0004000;
const IXOFF = 0o0010000;
const IMAXBEL = 0o0020000;
const IUTF8 = 0o0040000;

const VINTR = 0;
const VQUIT = 1;
const VERASE = 2;
const VKILL = 3;
const VEOF = 4;
const VTIME = 5;
const VMIN = 6;
const VSWTC = 7;
const VSTART = 8;
const VSTOP = 9;
const VSUSP = 10;
const VEOL = 11;
const VREPRINT = 12;
const VDISCARD = 13;
const VWERASE = 14;
const VLNEXT = 15;
const VEOL2 = 16;

/// This function configures a serial port with the given config.
/// `port` is an already opened serial port, on windows these
/// are either called `\\.\COMxx\` or `COMx`, on unixes the serial
/// port is called `/dev/ttyXXX`.
pub fn configureSerialPort(port: std.fs.File, config: SerialConfig) !void {
    switch (std.builtin.os.tag) {
        .windows => {
            var dcb = std.mem.zeroes(DCB);
            dcb.DCBlength = @sizeOf(DCB);

            if (GetCommState(port.handle, &dcb) == 0)
                return error.WindowsError;

            std.debug.warn("dcb = {}\n", .{dcb});

            dcb.BaudRate = config.baud_rate;
            dcb.fBinary = 1;
            dcb.fParity = if (config.parity != .none) @as(u1, 1) else @as(u1, 0);
            dcb.fOutxCtsFlow = if (config.handshake == .hardware) @as(u1, 1) else @as(u1, 0);
            dcb.fOutxDsrFlow = 0;
            dcb.fDtrControl = 0;
            dcb.fDsrSensitivity = 0;
            dcb.fTXContinueOnXoff = 0;
            dcb.fOutX = if (config.handshake == .software) @as(u1, 1) else @as(u1, 0);
            dcb.fInX = if (config.handshake == .software) @as(u1, 1) else @as(u1, 0);
            dcb.fErrorChar = 0;
            dcb.fNull = 0;
            dcb.fRtsControl = if (config.handshake == .hardware) @as(u1, 1) else @as(u1, 0);
            dcb.fAbortOnError = 0;
            dcb.wReserved = 0;
            dcb.ByteSize = config.word_size;
            dcb.Parity = switch (config.parity) {
                .none => @as(u8, 0),
                .even => @as(u8, 2),
                .odd => @as(u8, 1),
                .mark => @as(u8, 3),
                .space => @as(u8, 4),
            };
            dcb.StopBits = switch (config.stop_bits) {
                .one => @as(u2, 0),
                .two => @as(u2, 2),
            };
            dcb.XonChar = 0x11;
            dcb.XoffChar = 0x13;
            // dcb.ErrorChar = 0xFF;
            // dcb.EofChar = 0x00;
            // dcb.EvtChar = ;
            dcb.wReserved1 = 0;

            if (SetCommState(port.handle, &dcb) == 0)
                return error.WindowsError;
        },
        .linux => {
            var settings = try std.os.tcgetattr(port.handle);

            settings.iflag = 0;
            settings.oflag = 0;
            settings.cflag = CREAD;
            settings.lflag = 0;
            settings.ispeed = 0;
            settings.ospeed = 0;

            // settings.iflag &= ~@as(std.os.tcflag_t, IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
            // settings.oflag &= ~@as(std.os.tcflag_t, OPOST);
            // settings.lflag &= ~@as(std.os.tcflag_t, ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            // settings.cflag &= ~@as(std.os.tcflag_t, CSIZE | PARENB);
            // settings.cflag |= CS8;

            // settings.cflag &= ~@as(std.os.tcflag_t, PARODD | CMSPAR);
            switch (config.parity) {
                .none => {},
                .odd => settings.cflag |= PARODD,
                .even => {}, // even parity is default
                .mark => settings.cflag |= PARODD | CMSPAR,
                .space => settings.cflag |= CMSPAR,
            }
            if (config.parity != .none) {
                settings.iflag |= INPCK; // enable parity checking
                settings.cflag |= PARENB; // enable parity generation
            }
            // else {
            //     settings.iflag &= ~@as(std.os.tcflag_t, INPCK); // disable parity checking
            //     settings.cflag &= ~@as(std.os.tcflag_t, PARENB); // disable parity generation
            // }

            switch (config.handshake) {
                .none => settings.cflag |= CLOCAL,
                .software => settings.iflag |= IXON | IXOFF,
                .hardware => settings.cflag |= CRTSCTS,
            }

            switch (config.stop_bits) {
                .one => {},
                .two => settings.cflag |= CSTOPB,
            }

            switch (config.word_size) {
                5 => settings.cflag |= CS5,
                6 => settings.cflag |= CS6,
                7 => settings.cflag |= CS7,
                8 => settings.cflag |= CS8,
                else => return error.UnsupportedWordSize,
            }

            const baudmask = try mapBaudToLinuxEnum(config.baud_rate);
            settings.cflag &= ~@as(std.os.tcflag_t, CBAUD);
            settings.cflag |= baudmask;
            settings.ispeed = baudmask;
            settings.ospeed = baudmask;

            settings.cc[VMIN] = 1;
            settings.cc[VSTOP] = 0x13; // XOFF
            settings.cc[VSTART] = 0x11; // XON
            settings.cc[VTIME] = 0;

            try std.os.tcsetattr(port.handle, .NOW, settings);
        },
        else => @compileError("unsupported OS, please implement!"),
    }
}

/// Flushes the serial port `port`. If `input` is set, all pending data in
/// the receive buffer is flushed, if `output` is set all pending data in
/// the send buffer is flushed.
pub fn flushSerialPort(port: std.fs.File, input: bool, output: bool) !void {
    switch (std.builtin.os.tag) {
        .windows => {
            const success = if (input and output)
                PurgeComm(port.handle, PURGE_TXCLEAR | PURGE_RXCLEAR)
            else if (input)
                PurgeComm(port.handle, PURGE_RXCLEAR)
            else if (output)
                PurgeComm(port.handle, PURGE_TXCLEAR)
            else
                @as(std.os.windows.BOOL, 0);
            if (success == 0)
                return error.FlushError;
        },

        .linux => if (input and output)
            try tcflush(port.handle, TCIOFLUSH)
        else if (input)
            try tcflush(port.handle, TCIFLUSH)
        else if (output)
            try tcflush(port.handle, TCOFLUSH),

        else => @compileError("unsupported OS, please implement!"),
    }
}

const PURGE_RXABORT = 0x0002;
const PURGE_RXCLEAR = 0x0008;
const PURGE_TXABORT = 0x0001;
const PURGE_TXCLEAR = 0x0004;

extern "kernel32" fn PurgeComm(hFile: std.os.windows.HANDLE, dwFlags: std.os.windows.DWORD) std.os.windows.BOOL;

const TCIFLUSH = 0;
const TCOFLUSH = 1;
const TCIOFLUSH = 2;
const TCFLSH = 0x540B;

fn tcflush(fd: std.os.fd_t, mode: usize) !void {
    if (std.os.linux.syscall3(.ioctl, @bitCast(usize, @as(isize, fd)), TCFLSH, mode) != 0)
        return error.FlushError;
}

fn mapBaudToLinuxEnum(baudrate: usize) !std.os.speed_t {
    return switch (baudrate) {
        // from termios.h
        50 => 0o0000001,
        75 => 0o0000002,
        110 => 0o0000003,
        134 => 0o0000004,
        150 => 0o0000005,
        200 => 0o0000006,
        300 => 0o0000007,
        600 => 0o0000010,
        1200 => 0o0000011,
        1800 => 0o0000012,
        2400 => 0o0000013,
        4800 => 0o0000014,
        9600 => 0o0000015,
        19200 => 0o0000016,
        38400 => 0o0000017,
        // from termios-baud.h
        57600 => 0o0010001,
        115200 => 0o0010002,
        230400 => 0o0010003,
        460800 => 0o0010004,
        500000 => 0o0010005,
        576000 => 0o0010006,
        921600 => 0o0010007,
        1000000 => 0o0010010,
        1152000 => 0o0010011,
        1500000 => 0o0010012,
        2000000 => 0o0010013,
        2500000 => 0o0010014,
        3000000 => 0o0010015,
        3500000 => 0o0010016,
        4000000 => 0o0010017,
        else => error.UnsupportedBaudRate,
    };
}

const DCB = extern struct {
    DCBlength: std.os.windows.DWORD,
    BaudRate: std.os.windows.DWORD,
    fBinary: std.os.windows.DWORD, // u1
    fParity: std.os.windows.DWORD, // u1
    fOutxCtsFlow: std.os.windows.DWORD, // u1
    fOutxDsrFlow: std.os.windows.DWORD, // u1
    fDtrControl: std.os.windows.DWORD, // u2
    fDsrSensitivity: std.os.windows.DWORD,
    fTXContinueOnXoff: std.os.windows.DWORD,
    fOutX: std.os.windows.DWORD, // u1
    fInX: std.os.windows.DWORD, // u1
    fErrorChar: std.os.windows.DWORD, // u1
    fNull: std.os.windows.DWORD, // u1
    fRtsControl: std.os.windows.DWORD, // u2
    fAbortOnError: std.os.windows.DWORD, // u1
    fDummy2: std.os.windows.DWORD, // u17
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

extern "kernel32" fn GetCommState(hFile: std.os.windows.HANDLE, lpDCB: *DCB) std.os.windows.BOOL;
extern "kernel32" fn SetCommState(hFile: std.os.windows.HANDLE, lpDCB: *DCB) std.os.windows.BOOL;

test "basic configuration test" {
    var cfg = SerialConfig{
        .handshake = .none,
        .baud_rate = 9600,
        .parity = .none,
        .word_size = 8,
        .stop_bits = .one,
    };

    var port = try std.fs.cwd().openFile(
        if (std.builtin.os.tag == .windows) "\\\\.\\COM5" else "/dev/ttyUSB0", // if any, these will likely exist on a machine
        .{ .read = true, .write = true },
    );
    defer port.close();

    try configureSerialPort(port, cfg);
}

test "basic flush test" {
    var port = try std.fs.cwd().openFile(
        if (std.builtin.os.tag == .windows) "\\\\.\\COM5" else "/dev/ttyUSB0", // if any, these will likely exist on a machine
        .{ .read = true, .write = true },
    );
    defer port.close();

    try flushSerialPort(port, true, true);
    try flushSerialPort(port, true, false);
    try flushSerialPort(port, false, true);
    try flushSerialPort(port, false, false);
}
