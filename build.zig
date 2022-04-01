const std = @import("std");

const pkgs = struct {
    const serial = std.build.Pkg{
        .name = "serial",
        .path = .{ .path = "src/serial.zig" },
    };
};

pub fn build(b: *std.build.Builder) void {
    const mode = b.standardReleaseOptions();
    const target = b.standardTargetOptions(.{});

    const echo_exe = b.addExecutable("serial-echo", "examples/echo.zig");
    echo_exe.setTarget(target);
    echo_exe.setBuildMode(mode);
    echo_exe.addPackage(pkgs.serial);
    echo_exe.install();
}
