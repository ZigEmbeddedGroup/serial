const std = @import("std");
const builtin = @import("builtin");
const CallingConvention = std.builtin.CallingConvention;
const c = @cImport(@cInclude("termios.h"));

pub fn list() !PortIterator {
    return try PortIterator.init();
}

pub fn list_info() !InformationIterator {
    return try InformationIterator.init();
}

pub const PortIterator = switch (builtin.os.tag) {
    .windows => WindowsPortIterator,
    .linux => LinuxPortIterator,
    .macos => DarwinPortIterator,
    else => @compileError("OS is not supported for port iteration"),
};

pub const InformationIterator = switch (builtin.os.tag) {
    .windows => WindowsInformationIterator,
    .linux => LinuxInformationIterator,
    // .linux, .macos => @panic("'Port Information' not yet implemented for this OS"),
    else => @compileError("OS is not supported for information iteration"),
};

pub const SerialPortDescription = struct {
    file_name: []const u8,
    display_name: []const u8,
    driver: ?[]const u8,
};

pub const PortInformation = struct {
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

const HKEY = std.os.windows.HKEY;
const HWND = std.os.windows.HANDLE;
const HDEVINFO = std.os.windows.HANDLE;
const DEVINST = std.os.windows.DWORD;
const SP_DEVINFO_DATA = extern struct {
    cbSize: std.os.windows.DWORD,
    classGuid: std.os.windows.GUID,
    devInst: std.os.windows.DWORD,
    reserved: std.os.windows.ULONG_PTR,
};

const WindowsPortIterator = struct {
    const Self = @This();

    key: HKEY,
    index: u32,

    name: [256:0]u8 = undefined,
    name_size: u32 = 256,

    data: [256]u8 = undefined,
    filepath_data: [256]u8 = undefined,
    data_size: u32 = 256,

    pub fn init() !Self {
        const HKEY_LOCAL_MACHINE = @as(HKEY, @ptrFromInt(0x80000002));
        const KEY_READ = 0x20019;

        var self: Self = undefined;
        self.index = 0;
        if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM\\", 0, KEY_READ, &self.key) != 0)
            return error.WindowsError;

        return self;
    }

    pub fn deinit(self: *Self) void {
        _ = RegCloseKey(self.key);
        self.* = undefined;
    }

    pub fn next(self: *Self) !?SerialPortDescription {
        defer self.index += 1;

        self.name_size = 256;
        self.data_size = 256;

        return switch (RegEnumValueA(self.key, self.index, &self.name, &self.name_size, null, null, &self.data, &self.data_size)) {
            0 => SerialPortDescription{
                .file_name = try std.fmt.bufPrint(&self.filepath_data, "\\\\.\\{s}", .{self.data[0 .. self.data_size - 1]}),
                .display_name = self.data[0 .. self.data_size - 1],
                .driver = self.name[0..self.name_size],
            },
            259 => null,
            else => error.WindowsError,
        };
    }
};

const WindowsInformationIterator = struct {
    const Self = @This();

    index: std.os.windows.DWORD,
    device_info_set: HDEVINFO,

    port_buffer: [256:0]u8,
    sys_buffer: [256:0]u8,
    name_buffer: [256:0]u8,
    desc_buffer: [256:0]u8,
    man_buffer: [256:0]u8,
    serial_buffer: [256:0]u8,
    hw_id: [256:0]u8,

    const Property = enum(std.os.windows.DWORD) {
        SPDRP_DEVICEDESC = 0x00000000,
        SPDRP_MFG = 0x0000000B,
        SPDRP_FRIENDLYNAME = 0x0000000C,
    };

    // GUID taken from <devguid.h>
    const DIGCF_PRESENT = 0x00000002;
    const DIGCF_DEVICEINTERFACE = 0x00000010;
    const device_setup_tokens = .{
        .{ std.os.windows.GUID{ .Data1 = 0x4d36e978, .Data2 = 0xe325, .Data3 = 0x11ce, .Data4 = .{ 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18 } }, DIGCF_PRESENT },
        .{ std.os.windows.GUID{ .Data1 = 0x4d36e96d, .Data2 = 0xe325, .Data3 = 0x11ce, .Data4 = .{ 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18 } }, DIGCF_PRESENT },
        .{ std.os.windows.GUID{ .Data1 = 0x86e0d1e0, .Data2 = 0x8089, .Data3 = 0x11d0, .Data4 = .{ 0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73 } }, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE },
        .{ std.os.windows.GUID{ .Data1 = 0x2c7089aa, .Data2 = 0x2e0e, .Data3 = 0x11d1, .Data4 = .{ 0xb1, 0x14, 0x00, 0xc0, 0x4f, 0xc2, 0xaa, 0xe4 } }, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE },
    };

    pub fn init() !Self {
        var self: Self = undefined;
        self.index = 0;

        inline for (device_setup_tokens) |token| {
            const guid = token[0];
            const flags = token[1];

            self.device_info_set = SetupDiGetClassDevsW(
                &guid,
                null,
                null,
                flags,
            );

            if (self.device_info_set != std.os.windows.INVALID_HANDLE_VALUE) break;
        }

        if (self.device_info_set == std.os.windows.INVALID_HANDLE_VALUE) return error.WindowsError;

        return self;
    }

    pub fn deinit(self: *Self) void {
        _ = SetupDiDestroyDeviceInfoList(self.device_info_set);
        self.* = undefined;
    }

    pub fn next(self: *Self) !?PortInformation {
        var device_info_data: SP_DEVINFO_DATA = .{
            .cbSize = @sizeOf(SP_DEVINFO_DATA),
            .classGuid = std.mem.zeroes(std.os.windows.GUID),
            .devInst = 0,
            .reserved = 0,
        };

        if (SetupDiEnumDeviceInfo(self.device_info_set, self.index, &device_info_data) != std.os.windows.TRUE) {
            return null;
        }

        defer self.index += 1;

        var info: PortInformation = std.mem.zeroes(PortInformation);
        @memset(&self.hw_id, 0);

        // NOTE: have not handled if port startswith("LPT")
        var length = getPortName(&self.device_info_set, &device_info_data, &self.port_buffer);
        info.port_name = self.port_buffer[0..length];

        info.system_location = try std.fmt.bufPrint(&self.sys_buffer, "\\\\.\\{s}", .{info.port_name});

        length = deviceRegistryProperty(&self.device_info_set, &device_info_data, Property.SPDRP_FRIENDLYNAME, &self.name_buffer);
        info.friendly_name = self.name_buffer[0..length];

        length = deviceRegistryProperty(&self.device_info_set, &device_info_data, Property.SPDRP_DEVICEDESC, &self.desc_buffer);
        info.description = self.desc_buffer[0..length];

        length = deviceRegistryProperty(&self.device_info_set, &device_info_data, Property.SPDRP_MFG, &self.man_buffer);
        info.manufacturer = self.man_buffer[0..length];

        if (SetupDiGetDeviceInstanceIdA(
            self.device_info_set,
            &device_info_data,
            @ptrCast(&self.hw_id),
            255,
            null,
        ) == std.os.windows.TRUE) {
            length = @as(u32, @truncate(std.mem.indexOfSentinel(u8, 0, &self.hw_id)));
            info.hw_id = self.hw_id[0..length];

            length = parseSerialNumber(&self.hw_id, &self.serial_buffer) catch 0;
            if (length == 0) {
                length = getParentSerialNumber(device_info_data.devInst, &self.hw_id, &self.serial_buffer) catch 0;
            }
            info.serial_number = self.serial_buffer[0..length];
            info.vid = parseVendorId(&self.hw_id) catch 0;
            info.pid = parseProductId(&self.hw_id) catch 0;
        } else {
            return error.WindowsError;
        }

        return info;
    }

    fn getPortName(device_info_set: *const HDEVINFO, device_info_data: *SP_DEVINFO_DATA, port_name: [*]u8) std.os.windows.DWORD {
        const hkey: HKEY = SetupDiOpenDevRegKey(
            device_info_set.*,
            device_info_data,
            0x00000001, // #define DICS_FLAG_GLOBAL
            0,
            0x00000001, // #define DIREG_DEV,
            std.os.windows.KEY_READ,
        );

        defer {
            _ = std.os.windows.advapi32.RegCloseKey(hkey);
        }

        inline for (.{ "PortName", "PortNumber" }) |key_token| {
            var port_length: std.os.windows.DWORD = std.os.windows.NAME_MAX;
            var data_type: std.os.windows.DWORD = 0;

            const result = RegQueryValueExA(
                hkey,
                @as(std.os.windows.LPSTR, @ptrCast(@constCast(key_token))),
                null,
                &data_type,
                port_name,
                &port_length,
            );

            // if this is valid, return now
            if (result == 0 and port_length > 0) {
                return port_length;
            }
        }

        return 0;
    }

    fn deviceRegistryProperty(device_info_set: *const HDEVINFO, device_info_data: *SP_DEVINFO_DATA, property: Property, property_str: [*]u8) std.os.windows.DWORD {
        var data_type: std.os.windows.DWORD = 0;
        var bytes_required: std.os.windows.DWORD = std.os.windows.MAX_PATH;

        const result = SetupDiGetDeviceRegistryPropertyA(
            device_info_set.*,
            device_info_data,
            @intFromEnum(property),
            &data_type,
            property_str,
            std.os.windows.NAME_MAX,
            &bytes_required,
        );

        if (result == std.os.windows.FALSE) {
            std.debug.print("GetLastError: {}\n", .{std.os.windows.kernel32.GetLastError()});
            bytes_required = 0;
        }

        return bytes_required;
    }

    fn getParentSerialNumber(devinst: DEVINST, devid: []const u8, serial_number: [*]u8) !std.os.windows.DWORD {
        if (std.mem.startsWith(u8, devid, "FTDI")) {
            // Should not be called on "FTDI" so just return the serial number.
            return try parseSerialNumber(devid, serial_number);
        } else if (std.mem.startsWith(u8, devid, "USB")) {
            // taken from pyserial
            const max_usb_device_tree_traversal_depth = 5;
            const start_vidpid = std.mem.indexOf(u8, devid, "VID") orelse return error.WindowsError;
            const vidpid_slice = devid[start_vidpid .. start_vidpid + 17]; // "VIDxxxx&PIDxxxx"

            // keep looping over parent device to extract serial number if it contains the target VID and PID.
            var depth: u8 = 0;
            var child_inst: DEVINST = devinst;
            while (depth <= max_usb_device_tree_traversal_depth) : (depth += 1) {
                var parent_id: DEVINST = undefined;
                var local_buffer: [256:0]u8 = std.mem.zeroes([256:0]u8);

                if (CM_Get_Parent(&parent_id, child_inst, 0) != 0) return error.WindowsError;
                if (CM_Get_Device_IDA(parent_id, @ptrCast(&local_buffer), 256, 0) != 0) return error.WindowsError;
                defer child_inst = parent_id;

                if (!std.mem.containsAtLeast(u8, local_buffer[0..255], 1, vidpid_slice)) continue;

                const length = try parseSerialNumber(local_buffer[0..255], serial_number);
                if (length > 0) return length;
            }
        }

        return error.WindowsError;
    }

    fn parseSerialNumber(devid: []const u8, serial_number: [*]u8) !std.os.windows.DWORD {
        var delimiter: ?[]const u8 = undefined;

        if (std.mem.startsWith(u8, devid, "USB")) {
            delimiter = "\\&";
        } else if (std.mem.startsWith(u8, devid, "FTDI")) {
            delimiter = "\\+";
        } else {
            // What to do here?
            delimiter = null;
        }

        if (delimiter) |del| {
            var it = std.mem.tokenize(u8, devid, del);

            // throw away the start
            _ = it.next();
            while (it.next()) |segment| {
                if (std.mem.startsWith(u8, segment, "VID_")) continue;
                if (std.mem.startsWith(u8, segment, "PID_")) continue;

                // If "MI_{d}{d}", this is an interface number. The serial number will have to be
                // sourced from the parent node. Probably do not have to check all these conditions.
                if (segment.len == 5 and std.mem.eql(u8, "MI_", segment[0..3]) and std.ascii.isDigit(segment[3]) and std.ascii.isDigit(segment[4])) return 0;

                @memcpy(serial_number, segment);
                return @as(std.os.windows.DWORD, @truncate(segment.len));
            }
        }

        return error.WindowsError;
    }

    fn parseVendorId(devid: []const u8) !u16 {
        var delimiter: ?[]const u8 = undefined;

        if (std.mem.startsWith(u8, devid, "USB")) {
            delimiter = "\\&";
        } else if (std.mem.startsWith(u8, devid, "FTDI")) {
            delimiter = "\\+";
        } else {
            delimiter = null;
        }

        if (delimiter) |del| {
            var it = std.mem.tokenize(u8, devid, del);

            while (it.next()) |segment| {
                if (std.mem.startsWith(u8, segment, "VID_")) {
                    return try std.fmt.parseInt(u16, segment[4..], 16);
                }
            }
        }

        return error.WindowsError;
    }

    fn parseProductId(devid: []const u8) !u16 {
        var delimiter: ?[]const u8 = undefined;

        if (std.mem.startsWith(u8, devid, "USB")) {
            delimiter = "\\&";
        } else if (std.mem.startsWith(u8, devid, "FTDI")) {
            delimiter = "\\+";
        } else {
            delimiter = null;
        }

        if (delimiter) |del| {
            var it = std.mem.tokenize(u8, devid, del);

            while (it.next()) |segment| {
                if (std.mem.startsWith(u8, segment, "PID_")) {
                    return try std.fmt.parseInt(u16, segment[4..], 16);
                }
            }
        }

        return error.WindowsError;
    }
};

extern "advapi32" fn RegOpenKeyExA(
    key: HKEY,
    lpSubKey: std.os.windows.LPCSTR,
    ulOptions: std.os.windows.DWORD,
    samDesired: std.os.windows.REGSAM,
    phkResult: *HKEY,
) callconv(CallingConvention.winapi) std.os.windows.LSTATUS;
extern "advapi32" fn RegCloseKey(key: HKEY) callconv(CallingConvention.winapi) std.os.windows.LSTATUS;
extern "advapi32" fn RegEnumValueA(
    hKey: HKEY,
    dwIndex: std.os.windows.DWORD,
    lpValueName: std.os.windows.LPSTR,
    lpcchValueName: *std.os.windows.DWORD,
    lpReserved: ?*std.os.windows.DWORD,
    lpType: ?*std.os.windows.DWORD,
    lpData: [*]std.os.windows.BYTE,
    lpcbData: *std.os.windows.DWORD,
) callconv(CallingConvention.winapi) std.os.windows.LSTATUS;
extern "advapi32" fn RegQueryValueExA(
    hKey: HKEY,
    lpValueName: std.os.windows.LPSTR,
    lpReserved: ?*std.os.windows.DWORD,
    lpType: ?*std.os.windows.DWORD,
    lpData: ?[*]std.os.windows.BYTE,
    lpcbData: ?*std.os.windows.DWORD,
) callconv(CallingConvention.winapi) std.os.windows.LSTATUS;
extern "setupapi" fn SetupDiGetClassDevsW(
    classGuid: ?*const std.os.windows.GUID,
    enumerator: ?std.os.windows.PCWSTR,
    hwndParanet: ?HWND,
    flags: std.os.windows.DWORD,
) callconv(CallingConvention.winapi) HDEVINFO;
extern "setupapi" fn SetupDiEnumDeviceInfo(
    devInfoSet: HDEVINFO,
    memberIndex: std.os.windows.DWORD,
    device_info_data: *SP_DEVINFO_DATA,
) callconv(CallingConvention.winapi) std.os.windows.BOOL;
extern "setupapi" fn SetupDiDestroyDeviceInfoList(
    device_info_set: HDEVINFO,
) callconv(CallingConvention.winapi) std.os.windows.BOOL;
extern "setupapi" fn SetupDiOpenDevRegKey(
    device_info_set: HDEVINFO,
    device_info_data: *SP_DEVINFO_DATA,
    scope: std.os.windows.DWORD,
    hwProfile: std.os.windows.DWORD,
    keyType: std.os.windows.DWORD,
    samDesired: std.os.windows.REGSAM,
) callconv(CallingConvention.winapi) HKEY;
extern "setupapi" fn SetupDiGetDeviceRegistryPropertyA(
    hDevInfo: HDEVINFO,
    pSpDevInfoData: *SP_DEVINFO_DATA,
    property: std.os.windows.DWORD,
    propertyRegDataType: ?*std.os.windows.DWORD,
    propertyBuffer: ?[*]std.os.windows.BYTE,
    propertyBufferSize: std.os.windows.DWORD,
    requiredSize: ?*std.os.windows.DWORD,
) callconv(CallingConvention.winapi) std.os.windows.BOOL;
extern "setupapi" fn SetupDiGetDeviceInstanceIdA(
    device_info_set: HDEVINFO,
    device_info_data: *SP_DEVINFO_DATA,
    deviceInstanceId: *?std.os.windows.CHAR,
    deviceInstanceIdSize: std.os.windows.DWORD,
    requiredSize: ?*std.os.windows.DWORD,
) callconv(CallingConvention.winapi) std.os.windows.BOOL;
extern "cfgmgr32" fn CM_Get_Parent(
    pdnDevInst: *DEVINST,
    dnDevInst: DEVINST,
    ulFlags: std.os.windows.ULONG,
) callconv(CallingConvention.winapi) std.os.windows.DWORD;
extern "cfgmgr32" fn CM_Get_Device_IDA(
    dnDevInst: DEVINST,
    buffer: std.os.windows.LPSTR,
    bufferLen: std.os.windows.ULONG,
    ulFlags: std.os.windows.ULONG,
) callconv(CallingConvention.winapi) std.os.windows.DWORD;

const LinuxPortIterator = struct {
    const Self = @This();

    const root_dir = "/sys/class/tty";

    // ls -hal /sys/class/tty/*/device/driver

    dir: std.fs.Dir,
    iterator: std.fs.Dir.Iterator,

    full_path_buffer: [std.fs.max_path_bytes]u8 = undefined,
    driver_path_buffer: [std.fs.max_path_bytes]u8 = undefined,

    pub fn init() !Self {
        var dir = try std.fs.cwd().openDir(root_dir, .{ .iterate = true });
        errdefer dir.close();

        return Self{
            .dir = dir,
            .iterator = dir.iterate(),
        };
    }

    pub fn deinit(self: *Self) void {
        self.dir.close();
        self.* = undefined;
    }

    pub fn next(self: *Self) !?SerialPortDescription {
        while (true) {
            if (try self.iterator.next()) |entry| {
                // not a dir => we don't care
                var tty_dir = self.dir.openDir(entry.name, .{}) catch continue;
                defer tty_dir.close();

                // we need the device dir
                // no device dir =>  virtual device
                var device_dir = tty_dir.openDir("device", .{}) catch continue;
                defer device_dir.close();

                // We need the symlink for "driver"
                const link = device_dir.readLink("driver", &self.driver_path_buffer) catch continue;

                // full_path_buffer
                // driver_path_buffer

                var fba = std.heap.FixedBufferAllocator.init(&self.full_path_buffer);

                const path = try std.fs.path.join(fba.allocator(), &.{
                    "/dev/",
                    entry.name,
                });

                return SerialPortDescription{
                    .file_name = path,
                    .display_name = path,
                    .driver = std.fs.path.basename(link),
                };
            } else {
                return null;
            }
        }
        return null;
    }
};

const LinuxInformationIterator = struct {
    const Self = @This();

    const root_dir = "/sys/class/tty";

    index: u8,
    dir: std.fs.Dir,
    iterator: std.fs.Dir.Iterator,

    driver_path_buffer: [std.fs.max_path_bytes]u8 = undefined,
    sys_buffer: [256:0]u8 = undefined,
    desc_buffer: [256:0]u8 = undefined,
    man_buffer: [256:0]u8 = undefined,
    serial_buffer: [256:0]u8 = undefined,
    port: PortInformation = undefined,

    pub fn init() !Self {
        var dir = try std.fs.cwd().openDir(root_dir, .{ .iterate = true });
        errdefer dir.close();

        return Self{ .index = 0, .dir = dir, .iterator = dir.iterate() };
    }

    pub fn deinit(self: *Self) void {
        self.dir.close();
        self.* = undefined;
    }

    pub fn next(self: *Self) !?PortInformation {
        self.index += 1;
        while (try self.iterator.next()) |entry| {
            @memset(&self.sys_buffer, 0);
            @memset(&self.desc_buffer, 0);
            @memset(&self.man_buffer, 0);
            @memset(&self.serial_buffer, 0);
            @memset(&self.driver_path_buffer, 0);

            // not a dir => we don't care
            var tty_dir = self.dir.openDir(entry.name, .{}) catch continue;
            defer tty_dir.close();

            // we need the device dir
            // no device dir =>  virtual device
            var device_dir = tty_dir.openDir("device", .{}) catch continue;
            defer device_dir.close();

            // start filling port informations
            {
                var fba = std.heap.FixedBufferAllocator.init(&self.sys_buffer);
                self.port.system_location = try std.fs.path.join(fba.allocator(), &.{
                    "/dev/",
                    entry.name,
                });
                self.port.friendly_name = entry.name;
                self.port.port_name = entry.name;
                self.port.hw_id = "N/A";
            }
            // We need the symlink for "driver"
            const subsystem_path = device_dir.readLink("subsystem", &self.driver_path_buffer) catch continue;
            const subsystem = std.fs.path.basename(subsystem_path);
            var device_path: []u8 = undefined;
            if (std.mem.eql(u8, subsystem, "usb") == true) {
                device_path = try device_dir.realpath("../", &self.driver_path_buffer);
            } else if (std.mem.eql(u8, subsystem, "usb-serial") == true) {
                device_path = try device_dir.realpath("../../", &self.driver_path_buffer);
            } else {
                //must be remove to manage other device type
                self.port.description = "Not Managed";
                self.port.manufacturer = "Not Managed";
                self.port.serial_number = "Not Managed";
                self.port.vid = 0;
                self.port.pid = 0;
                return self.port;
            }

            var data_dir = std.fs.openDirAbsolute(device_path, .{}) catch continue;
            defer data_dir.close();
            var tmp: [4]u8 = undefined;
            {
                self.port.manufacturer = data_dir.readFile("manufacturer", &self.man_buffer) catch "N/A";
                Self.clean_file_read(&self.man_buffer);
                self.port.description = data_dir.readFile("product", &self.desc_buffer) catch "N/A";
                Self.clean_file_read(&self.desc_buffer);
                self.port.serial_number = data_dir.readFile("serial", &self.serial_buffer) catch "N/A";
                Self.clean_file_read(&self.serial_buffer);
            }
            {
                @memset(&tmp, 0);
                _ = data_dir.readFile("idVendor", &tmp) catch 0;
                self.port.vid = try std.fmt.parseInt(u16, &tmp, 16);
            }
            {
                @memset(&tmp, 0);
                _ = data_dir.readFile("idProduct", &tmp) catch 0;
                self.port.pid = try std.fmt.parseInt(u16, &tmp, 16);
            }

            return self.port;
        }
        return null;
    }
    fn clean_file_read(buf: []u8) void {
        for (buf) |*item| {
            if (item.* == '\n') {
                item.* = 0;
                break;
            }
        }
    }
};

const DarwinPortIterator = struct {
    const Self = @This();

    const root_dir = "/dev/";

    dir: std.fs.Dir,
    iterator: std.fs.Dir.Iterator,

    full_path_buffer: [std.fs.max_path_bytes]u8 = undefined,
    driver_path_buffer: [std.fs.max_path_bytes]u8 = undefined,

    pub fn init() !Self {
        var dir = try std.fs.cwd().openDir(root_dir, .{ .iterate = true });
        errdefer dir.close();

        return Self{
            .dir = dir,
            .iterator = dir.iterate(),
        };
    }

    pub fn deinit(self: *Self) void {
        self.dir.close();
        self.* = undefined;
    }

    pub fn next(self: *Self) !?SerialPortDescription {
        while (true) {
            if (try self.iterator.next()) |entry| {
                if (!std.mem.startsWith(u8, entry.name, "cu.")) {
                    continue;
                } else {
                    var fba = std.heap.FixedBufferAllocator.init(&self.full_path_buffer);

                    const path = try std.fs.path.join(fba.allocator(), &.{
                        "/dev/",
                        entry.name,
                    });

                    return SerialPortDescription{
                        .file_name = path,
                        .display_name = path,
                        .driver = "darwin",
                    };
                }
            } else {
                return null;
            }
        }
        return null;
    }
};

pub const Parity = enum(u8) {
    /// No parity bit is used
    none = 'N',
    /// Parity bit is `0` when an even number of bits is set in the data.
    even = 'E',
    /// Parity bit is `0` when an odd number of bits is set in the data.
    odd = 'O',
    /// Parity bit is always `1`
    mark = 'M',
    /// Parity bit is always `0`
    space = 'S',
};

pub const StopBits = enum(u2) {
    /// The length of the stop bit is 1 bit
    one = 1,
    /// The length of the stop bit is 2 bits
    two = 2,
};

pub const Handshake = enum {
    /// No handshake is used
    none,
    /// XON-XOFF software handshake is used.
    software,
    /// Hardware handshake with RTS/CTS is used.
    hardware,
};

pub const WordSize = enum(u4) {
    five = 5,
    six = 6,
    seven = 7,
    eight = 8,
};

pub const SerialConfig = struct {
    const Self = @This();

    /// Symbol rate in bits/second. Not that these
    /// include also parity and stop bits.
    baud_rate: u32,

    /// Parity to verify transport integrity.
    parity: Parity = .none,

    /// Number of stop bits after the data
    stop_bits: StopBits = .one,

    /// Number of data bits per word.
    /// Allowed values are 5, 6, 7, 8
    word_size: WordSize = .eight,

    /// Defines the handshake protocol used.
    handshake: Handshake = .none,

    /// Timeout. Max timeout for posix is 25.5 seconds/255 deciseconds
    timeout: ?Timeout = null,

    pub fn format(self: Self, fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = options;
        _ = fmt;
        return writer.print("{d}@{d}{c}{d}{s}", .{
            self.baud_rate,
            @intFromEnum(self.word_size),
            @intFromEnum(self.parity),
            @intFromEnum(self.stop_bits),
            switch (self.handshake) {
                .none => "",
                .hardware => " RTS/CTS",
                .software => " XON/XOFF",
            },
        });
    }
};

/// Max timeout for posix is 25500 milliseconds/255 deciseconds
/// Windows uses a u32 for milliseconds, posix uses u8 for deciseconds
/// ms will be converted to deciseconds if used on posix.
pub const Timeout = union(enum) {
    /// milliseconds
    ms: u32,

    /// deciseconds for posix
    ds: u8,
};

const CBAUD = 0o000000010017; //Baud speed mask (not in POSIX).
const CMSPAR = 0o010000000000;
const CRTSCTS = 0o020000000000;

/// This function configures a serial port with the given config.
/// `port` is an already opened serial port, on windows these
/// are either called `\\.\COMxx\` or `COMx`, on unixes the serial
/// port is called `/dev/ttyXXX`.
pub fn configureSerialPort(port: std.fs.File, config: SerialConfig) !void {
    if (builtin.os.tag == .windows) {
        var dcb = std.mem.zeroes(DCB);
        dcb.DCBlength = @sizeOf(DCB);

        if (GetCommState(port.handle, &dcb) == 0)
            return error.WindowsError;

        // std.log.err("{s} {s}", .{ dcb, flags });

        dcb.BaudRate = config.baud_rate;

        dcb.flags = @bitCast(DCBFlags{
            .fParity = config.parity != .none,
            .fOutxCtsFlow = config.handshake == .hardware,
            .fOutX = config.handshake == .software,
            .fInX = config.handshake == .software,
            .fRtsControl = @as(u2, if (config.handshake == .hardware) 1 else 0),
        });

        dcb.wReserved = 0;
        dcb.ByteSize = switch (config.word_size) {
            .five => @as(u8, 5),
            .six => @as(u8, 6),
            .seven => @as(u8, 7),
            .eight => @as(u8, 8),
        };
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
        dcb.wReserved1 = 0;

        if (SetCommState(port.handle, &dcb) == 0)
            return error.WindowsError;

        if (config.timeout == null) return;

        var timeouts: COMMTIMEOUTS = std.mem.zeroes(COMMTIMEOUTS);
        if (FALSE == GetCommTimeouts(handle, &timeouts))
            return error.Configuration;

        // Convert the timeout value based on the union type
        const timeout_ms = switch (config.timeout.?) {
            .ms => |ms| ms,
            .ds => |ds| @as(u32, ds) * 100, // Convert deciseconds to milliseconds
        };

        if (timeout_ms == 0) {
            const MAXDWORD: DWORD = 0xFFFFFFFF;
            // Return immediately with any characters received
            timeouts.ReadIntervalTimeout = MAXDWORD;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 0;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 0;
        } else {
            timeouts.ReadIntervalTimeout = 0;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = timeout_ms;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = timeout_ms;
        }

        if (FALSE == SetCommTimeouts(handle, &timeouts))
            return error.Configuration;
        return;
    }
    // posix config
    var settings = try std.posix.tcgetattr(port.handle);

    const macos_nonstandard_baud = false;
    const speed_t = getSpeed(config.baud_rate) orelse blk: {
        if (builtin.os.tag == .macos) {
            macos_nonstandard_baud = true;
            break :blk std.posix.speed_t.B9600;
            // just filling it with anything to appease the compiler, it won't be used.
        }
        return error.UnsupportedBaudRate;
    };
    const parity = config.parity;
    const stop_bits = config.stop_bits;
    const line_ending = config.line_ending;
    const word_size: std.posix.CSIZE = switch (config.word_size) {
        .five => .CS5,
        .six => .CS6,
        .seven => .CS7,
        .eight => .CS8,
    };

    settings.iflag = .{};
    safeSet(settings.iflag, "IGNPAR", parity == .none);
    safeSet(settings.iflag, "PARMRK", parity != .none);
    safeSet(settings.iflag, "INPCK", parity == .none);
    safeSet(settings.iflag, "IXON", handshake == .software);
    safeSet(settings.iflag, "IXOFF", handshake == .software);
    safeSet(settings.iflag, "IXANY", handshake == .software);
    safeSet(settings.iflag, "INLCR", false);
    safeSet(settings.iflag, "ICRNL", false);
    safeSet(settings.iflag, "IGNBRK", false);
    safeSet(settings.iflag, "BRKINT", false);
    safeSet(settings.iflag, "ISTRIP", false);
    safeSet(settings.iflag, "IGNCR", false);
    safeSet(settings.iflag, "IMAXBEL", false);
    safeSet(settings.iflag, "IUTF8", false);
    safeSet(settings.iflag, "IUCLC", false);
    safeSet(settings.iflag, "DOSMODE", false);
    settings.cflag = .{};
    safeSet(settings.cflag, "PARENB", parity != .none);
    safeSet(settings.cflag, "CMSPAR", parity == .mark);
    safeSet(settings.cflag, "PARODD", parity == .odd or parity == .mark);
    safeSet(settings.cflag, "PAREXT", parity == .mark or parity == .space);
    safeSet(settings.cflag, "CLOCAL", handshake == .none);
    safeSet(settings.cflag, "CNO_RTSDTR", handshake == .none);
    safeSet(settings.cflag, "HUPCL", handshake == .hardware);
    safeSet(settings.cflag, "MDMBUF", handshake == .hardware);
    safeSet(settings.cflag, "CRTSCTS", handshake == .hardware);
    safeSet(settings.cflag, "CTSFLOW", handshake == .hardware);
    safeSet(settings.cflag, "RTSFLOW", handshake == .hardware);
    safeSet(settings.cflag, "CDTRCTS", handshake == .hardware);
    safeSet(settings.cflag, "CRTSXOFF", handshake == .hardware);
    safeSet(settings.cflag, "CCTS_OFLOW", handshake == .hardware);
    safeSet(settings.cflag, "CRTS_IFLOW", handshake == .hardware);
    safeSet(settings.cflag, "CDTR_IFLOW", handshake == .hardware);
    safeSet(settings.cflag, "CDSR_OFLOW", handshake == .hardware);
    safeSet(settings.cflag, "CCAR_OFLOW", handshake == .hardware);
    safeSet(settings.cflag, "CSTOPB", stop_bits == .two);
    safeSet(settings.cflag, "CSIZE", word_size);
    safeSet(settings.cflag, "CREAD", true);
    safeSet(settings.cflag, "CIGNORE", false);
    safeSet(settings.cflag, "CBAUDEXT", baud > 38400);
    safeSet(settings.cflag, "CIBAUDEXT", baud > 38400);
    safeSet(settings.cflag, "LOBLK", false);
    safeSet(settings.cflag, "ADDRB", false);
    safeSet(settings.cflag, "XLOBLK", false);
    safeSet(settings.cflag, "RCV1EN", false);
    safeSet(settings.cflag, "XMT1EN", false);
    safeSet(settings.cflag, "XCLUDE", false);
    settings.lflag = .{};
    settings.oflag = .{};

    if (@hasField(settings, "ispeed")) {
        if (!macos_nonstandard_baud) settings.ispeed = speed_t;
    }
    if (@hasField(settings, "ospeed")) {
        if (!macos_nonstandard_baud) settings.ospeed = speed_t;
    }
    if (@hasField(settings, "line")) {
        // I don't have a system with this, leaving it here for now
    }

    if (config.timeout) |timeout| {
        settings.cc[std.posix.V.MIN] = 0;
        settings.cc[std.posix.V.TIME] = switch (timeout) {
            .ds => |ds| ds,
            .ms => |ms| blk: {
                const deciseconds = (ms + 99) / 100;
                break :blk if (deciseconds > 255) 255 else @as(u8, @intCast(deciseconds));
            },
        };
    } else {
        settings.cc[std.posix.V.MIN] = 1;
        settings.cc[std.posix.V.TIME] = 0;
    }
    settings.cc[std.posix.V.STOP] = 0x13; // XOFF
    settings.cc[std.posix.V.START] = 0x11; // XON

    try std.posix.tcsetattr(port.handle, .NOW, settings);

    if (macos_nonstandard_baud) {
        const IOSSIOSPEED: c_uint = 0x80085402;
        const speed: c_uint = @intCast(config.baud_rate);
        if (-1 == std.c.ioctl(port.handle, @bitCast(IOSSIOSPEED), &speed))
            return error.UnsupportedBaudRate;
    }
}

/// Checks to see if the field exists before setting it.
fn safeSet(container: anytype, field: []const u8, value: anytype) void {
    if (@hasField(@TypeOf(container), field))
        @field(container, field) = value;
}

fn getSpeed(baudrate: u32) ?std.posix.speed_t {
    return std.meta.intToEnum(std.posix.speed_t, baudrate) catch null;
}

const Flush = enum {
    input,
    output,
    both,
};

/// Flushes the serial port `port`. If `input` is set, all pending data in
/// the receive buffer is flushed, if `output` is set all pending data in
/// the send buffer is flushed.
pub fn flushSerialPort(port: std.fs.File, flush: Flush) !void {
    switch (builtin.os.tag) {
        .windows => {
            const mode: std.os.windows.DWORD = switch (flush) {
                .input => PURGE_RXCLEAR,
                .output => PURGE_TXCLEAR,
                .both => PURGE_TXCLEAR | PURGE_RXCLEAR,
            };
            if (0 == PurgeComm(port.handle, mode))
                return error.FlushError;
        },
        .linux => {
            const TCFLSH = 0x540B;
            const mode: usize = switch (flush) {
                .input => 0, // TCIFLUSH
                .output => 1, // TCOFLUSH
                .both => 2, // TCIOFLUSH
            };
            if (0 != std.os.linux.syscall3(.ioctl, @as(usize, @bitCast(@as(isize, port.handle))), TCFLSH, mode))
                return error.FlushError;
        },
        .macos => {
            const mode: c_int = switch (flush) {
                .input => c.TCIFLUSH,
                .output => c.TCOFLUSH,
                .both => c.TCIOFLUSH,
            };
            if (0 != c.tcflush(port.handle, mode))
                return error.FlushError;
        },
        else => @compileError("unsupported OS, please implement!"),
    }
}

pub const ControlPins = struct {
    rts: ?bool = null,
    dtr: ?bool = null,
};

pub fn changeControlPins(port: std.fs.File, pins: ControlPins) !void {
    switch (builtin.os.tag) {
        .windows => {
            const CLRDTR = 6;
            const CLRRTS = 4;
            const SETDTR = 5;
            const SETRTS = 3;

            if (pins.dtr) |dtr| {
                if (EscapeCommFunction(port.handle, if (dtr) SETDTR else CLRDTR) == 0)
                    return error.WindowsError;
            }
            if (pins.rts) |rts| {
                if (EscapeCommFunction(port.handle, if (rts) SETRTS else CLRRTS) == 0)
                    return error.WindowsError;
            }
        },
        .linux => {
            const TIOCM_RTS: c_int = 0x004;
            const TIOCM_DTR: c_int = 0x002;

            // from /usr/include/asm-generic/ioctls.h
            // const TIOCMBIS: u32 = 0x5416;
            // const TIOCMBIC: u32 = 0x5417;
            const TIOCMGET: u32 = 0x5415;
            const TIOCMSET: u32 = 0x5418;

            var flags: c_int = 0;
            if (std.os.linux.ioctl(port.handle, TIOCMGET, @intFromPtr(&flags)) != 0)
                return error.Unexpected;

            if (pins.dtr) |dtr| {
                if (dtr) {
                    flags |= TIOCM_DTR;
                } else {
                    flags &= ~TIOCM_DTR;
                }
            }
            if (pins.rts) |rts| {
                if (rts) {
                    flags |= TIOCM_RTS;
                } else {
                    flags &= ~TIOCM_RTS;
                }
            }

            if (std.os.linux.ioctl(port.handle, TIOCMSET, @intFromPtr(&flags)) != 0)
                return error.Unexpected;
        },

        .macos => {},

        else => @compileError("changeControlPins not implemented for " ++ @tagName(builtin.os.tag)),
    }
}

const PURGE_RXABORT = 0x0002;
const PURGE_RXCLEAR = 0x0008;
const PURGE_TXABORT = 0x0001;
const PURGE_TXCLEAR = 0x0004;

extern "kernel32" fn PurgeComm(
    in_hFile: std.os.windows.HANDLE,
    in_dwFlags: std.os.windows.DWORD,
) callconv(CallingConvention.winapi) std.os.windows.BOOL;

extern "kernel32" fn EscapeCommFunction(
    in_hFile: HANDLE,
    in_dwFunc: DWORD,
) callconv(CallingConvention.winapi) std.os.windows.BOOL;

extern "kernel32" fn GetCommTimeouts(
    in_hFile: std.os.windows.HANDLE,
    out_lpCommTimeouts: *COMMTIMEOUTS,
) callconv(CallingConvention.winapi) std.os.windows.BOOL;

extern "kernel32" fn SetCommTimeouts(
    in_hFile: std.os.windows.HANDLE,
    in_lpCommTimeouts: *COMMTIMEOUTS,
) callconv(CallingConvention.winapi) std.os.windows.BOOL;

const COMMTIMEOUTS = extern struct {
    ReadIntervalTimeout: DWORD,
    ReadTotalTimeoutMultiplier: DWORD,
    ReadTotalTimeoutConstant: DWORD,
    WriteTotalTimeoutMultiplier: DWORD,
    WriteTotalTimeoutConstant: DWORD,
};

const DCBFlags = packed struct(u32) {
    fBinary: bool = true, // u1
    fParity: bool = false, // u1
    fOutxCtsFlow: bool = false, // u1
    fOutxDsrFlow: bool = false, // u1
    fDtrControl: u2 = 1, // u2
    fDsrSensitivity: bool = false, // u1
    fTXContinueOnXoff: bool = false, // u1
    fOutX: bool = false, // u1
    fInX: bool = false, // u1
    fErrorChar: bool = false, // u1
    fNull: bool = false, // u1
    fRtsControl: u2 = 0, // u2
    fAbortOnError: bool = false, // u1
    fDummy2: u17 = 0, // u17
};

/// Configuration for the serial port
///
/// Details: https://learn.microsoft.com/es-es/windows/win32/api/winbase/ns-winbase-dcb
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

extern "kernel32" fn GetCommState(
    in_hFile: std.os.windows.HANDLE,
    in_out_lpDCB: *DCB,
) callconv(CallingConvention.winapi) std.os.windows.BOOL;

extern "kernel32" fn SetCommState(
    in_hFile: std.os.windows.HANDLE,
    in_lpDCB: *DCB,
) callconv(CallingConvention.winapi) std.os.windows.BOOL;

test "iterate ports" {
    var it = try list();
    while (try it.next()) |port| {
        _ = port;
        // std.debug.print("{s} (file: {s}, driver: {s})\n", .{ port.display_name, port.file_name, port.driver });
    }
}

test "basic configuration test" {
    const cfg = SerialConfig{
        .handshake = .none,
        .baud_rate = 115200,
        .parity = .none,
        .word_size = .eight,
        .stop_bits = .one,
    };

    var tty: []const u8 = undefined;

    switch (builtin.os.tag) {
        .windows => tty = "\\\\.\\COM3",
        .linux => tty = "/dev/ttyUSB0",
        .macos => tty = "/dev/cu.usbmodem101",
        else => unreachable,
    }

    var port = try std.fs.cwd().openFile(tty, .{ .mode = .read_write });
    defer port.close();

    try configureSerialPort(port, cfg);
}

test "basic flush test" {
    var tty: []const u8 = undefined;
    // if any, these will likely exist on a machine
    switch (builtin.os.tag) {
        .windows => tty = "\\\\.\\COM3",
        .linux => tty = "/dev/ttyUSB0",
        .macos => tty = "/dev/cu.usbmodem101",
        else => unreachable,
    }
    var port = try std.fs.cwd().openFile(tty, .{ .mode = .read_write });
    defer port.close();

    try flushSerialPort(port, .both);
    try flushSerialPort(port, .input);
    try flushSerialPort(port, .output);
}

test "change control pins" {
    _ = changeControlPins;
}

test "bufPrint tests" {
    var buf: [32]u8 = undefined;

    try std.testing.expect(std.mem.eql(u8, try std.fmt.bufPrint(&buf, "{}", .{SerialConfig{
        .handshake = .software,
        .baud_rate = 115200,
        .parity = .none,
        .word_size = .eight,
        .stop_bits = .one,
    }}), "115200@8N1 XON/XOFF"));

    try std.testing.expect(std.mem.eql(u8, try std.fmt.bufPrint(&buf, "{}", .{SerialConfig{
        .handshake = .hardware,
        .baud_rate = 115200,
        .parity = .none,
        .word_size = .eight,
        .stop_bits = .one,
    }}), "115200@8N1 RTS/CTS"));

    try std.testing.expect(std.mem.eql(u8, try std.fmt.bufPrint(&buf, "{}", .{SerialConfig{
        .handshake = .none,
        .baud_rate = 115200,
        .parity = .none,
        .word_size = .eight,
        .stop_bits = .one,
    }}), "115200@8N1"));

    try std.testing.expect(std.mem.eql(u8, try std.fmt.bufPrint(&buf, "{}", .{SerialConfig{
        .handshake = .none,
        .baud_rate = 115200,
        .parity = .even,
        .word_size = .eight,
        .stop_bits = .one,
    }}), "115200@8E1"));

    try std.testing.expect(std.mem.eql(u8, try std.fmt.bufPrint(&buf, "{}", .{SerialConfig{
        .handshake = .none,
        .baud_rate = 115200,
        .parity = .odd,
        .word_size = .eight,
        .stop_bits = .one,
    }}), "115200@8O1"));

    try std.testing.expect(std.mem.eql(u8, try std.fmt.bufPrint(&buf, "{}", .{SerialConfig{
        .handshake = .none,
        .baud_rate = 115200,
        .parity = .space,
        .word_size = .eight,
        .stop_bits = .one,
    }}), "115200@8S1"));

    try std.testing.expect(std.mem.eql(u8, try std.fmt.bufPrint(&buf, "{}", .{SerialConfig{
        .handshake = .none,
        .baud_rate = 115200,
        .parity = .mark,
        .word_size = .eight,
        .stop_bits = .one,
    }}), "115200@8M1"));

    try std.testing.expect(std.mem.eql(u8, try std.fmt.bufPrint(&buf, "{}", .{SerialConfig{
        .handshake = .none,
        .baud_rate = 9600,
        .parity = .mark,
        .word_size = .five,
        .stop_bits = .one,
    }}), "9600@5M1"));
}
