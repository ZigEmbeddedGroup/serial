const std = @import("std");

const pkgs = struct {
    const serial = std.build.Pkg{
        .name = "serial",
        .source = .{ .path = "src/serial.zig" },
    };
};

pub fn build(b: *std.build.Builder) void {
    const mode = b.standardReleaseOptions();
    const target = b.standardTargetOptions(.{});

    const echo_exe = b.addExecutable("serial-echo", "examples/echo.zig");
    echo_exe.setTarget(target);
    echo_exe.setBuildMode(mode);
    echo_exe.addPackage(pkgs.serial);
    if (target.isDarwin())
        echo_exe.linkLibC();
    echo_exe.install();

    const list_exe = b.addExecutable("serial-list", "examples/list.zig");
    list_exe.setTarget(target);
    list_exe.setBuildMode(mode);
    list_exe.addPackage(pkgs.serial);
    if (target.isDarwin())
        echo_exe.linkLibC();
    list_exe.install();
}
