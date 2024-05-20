dir: std.fs.IterableDir,
iterator: std.fs.IterableDir.Iterator,

full_path_buffer: [std.fs.max_path_bytes]u8 = undefined,
driver_path_buffer: [std.fs.max_path_bytes]u8 = undefined,

pub fn init() !Iterator {
    var dir = try std.fs.cwd().openIterableDir(root_dir, .{});
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
        if (try self.iterator.next()) |entry| { // couldn't this be placed inside while ()?
            if (!std.mem.startsWith(u8, entry.name, "cu.")) {
                continue;
            } else {
                var fba = std.heap.FixedBufferAllocator.init(&self.full_path_buffer);

                const path = try std.fs.path.join(fba.allocator(), &.{
                    "/dev/",
                    entry.name,
                });

                return common_serial.Description{
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

const Iterator = @This();
const std = @import("std");
const common_serial = @import("../common_serial.zig");
const root_dir = "/dev/";