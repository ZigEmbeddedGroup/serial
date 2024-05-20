key: std.os.windows.HKEY,
index: u32,
name: [256:0]u8 = undefined,

name_size: u32 = 256,

data: [256]u8 = undefined,
filepath_data: [256]u8 = undefined,
data_size: u32 = 256,

pub fn init() !Iterator {
    const HKEY_LOCAL_MACHINE = @as(
        std.os.windows.HKEY,
        @ptrFromInt(0x80000002),
    );
    const KEY_READ = 0x20019;

    var self: Iterator = undefined;
    self.index = 0;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM\\", 0, KEY_READ, &self.key) != 0)
        return error.iterator_error;

    return self;
}

pub fn deinit(self: *Iterator) void {
    _ = RegCloseKey(self.key);
    self.* = undefined;
}

pub fn next(self: *Iterator) !?common_serial.Description {
    defer self.index += 1;

    self.name_size = 256;
    self.data_size = 256;

    return switch (RegEnumValueA(self.key, self.index, &self.name, &self.name_size, null, null, &self.data, &self.data_size)) {
        0 => common_serial.Description{
            .file_name = try std.fmt.bufPrint(&self.filepath_data, "\\\\.\\{s}", .{self.data[0 .. self.data_size - 1]}),
            .display_name = self.data[0 .. self.data_size - 1],
            .driver = self.name[0..self.name_size],
        },
        259 => null,
        else => error.iterator_error,
    };
}

const Iterator = @This();
const std = @import("std");
const common_serial = @import("../common_serial.zig");

extern "advapi32" fn RegOpenKeyExA(
    key: std.os.windows.HKEY,
    lpSubKey: std.os.windows.LPCSTR,
    ulOptions: std.os.windows.DWORD,
    samDesired: std.os.windows.REGSAM,
    phkResult: *std.os.windows.HKEY,
) callconv(std.os.windows.WINAPI) std.os.windows.LSTATUS;

extern "advapi32" fn RegCloseKey(
    key: std.os.windows.HKEY,
) callconv(std.os.windows.WINAPI) std.os.windows.LSTATUS;

extern "advapi32" fn RegEnumValueA(
    hKey: std.os.windows.HKEY,
    dwIndex: std.os.windows.DWORD,
    lpValueName: std.os.windows.LPSTR,
    lpcchValueName: *std.os.windows.DWORD,
    lpReserved: ?*std.os.windows.DWORD,
    lpType: ?*std.os.windows.DWORD,
    lpData: [*]std.os.windows.BYTE,
    lpcbData: *std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) std.os.windows.LSTATUS;