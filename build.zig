const std = @import("std");
const log = std.log.scoped(.serial_lib__build);

const example_files = [_][]const u8{
    "echo",
    "list",
    "list_port_info",
};

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const serial_mod = b.addModule("serial", .{
        .root_source_file = b.path("src/serial.zig"),
    });

    const unit_tests = b.addTest(.{
        .root_source_file = b.path("src/serial.zig"),
        .target = target,
        .optimize = optimize,
    });
    const run_unit_tests = b.addRunArtifact(unit_tests);
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_unit_tests.step);

    const example_step = b.step("examples", "Build examples");
    {
        for (example_files) |example_name| {
            const example = b.addExecutable(.{
                .name = example_name,
                .root_source_file = b.path(
                    b.fmt("examples/{s}.zig", .{example_name}),
                ),
                .target = target,
                .optimize = optimize,
            });

            // TODO: Linux and MacOS port info support

            example.root_module.addImport("serial", serial_mod);
            const install_example = b.addInstallArtifact(example, .{});
            example_step.dependOn(&example.step);
            example_step.dependOn(&install_example.step);
        }
    }
}
