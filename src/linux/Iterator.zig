dir: std.fs.Dir,
iterator: std.fs.Dir.Iterator,

full_path_buffer: [std.fs.max_path_bytes]u8 = undefined,
driver_path_buffer: [std.fs.max_path_bytes]u8 = undefined,

// ls -hal /sys/class/tty/*/device/driver


pub fn init() !Iterator {
    var dir = try std.fs.cwd().openDir(root_dir, .{ .iterate = true });
    errdefer dir.close();

    return .{
        .dir = dir,
        .iterator = dir.iterate(),
    };
}

pub fn deinit(self: *Iterator) void {
    self.dir.close();
    self.* = undefined;
}

pub fn next(self: *Iterator) !?common_serial.Description {
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

            return common_serial.Description{
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

const Iterator = @This();
const std = @import("std");
const common_serial = @import("../common_serial.zig");
const root_dir = "/sys/class/tty";