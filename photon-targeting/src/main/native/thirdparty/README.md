# Vendored third-party code for the GPU AprilTag detector

| Directory | Upstream | Version | License |
| --- | --- | --- | --- |
| `vkapriltag/` | <https://github.com/yojobama/vkapriltag> (library sources and shaders only) | see `vkapriltag/VERSION` | Apache-2.0 (`vkapriltag/LICENSE`, `vkapriltag/NOTICE`) |
| `apriltag3/` | <https://github.com/AprilRobotics/apriltag>, with vkapriltag's patches applied (they expose the decode steps) | see `apriltag3/VERSION` | BSD-2-Clause (`apriltag3/LICENSE.md`) |
| `vulkan-headers/` | <https://github.com/KhronosGroup/Vulkan-Headers> (C headers only) | see `vulkan-headers/VERSION` | Apache-2.0 |

Don't edit these files by hand. Re-vendor with `scripts/vendor_vkapriltag.sh <built vkapriltag checkout>`,
which also regenerates `../gpu/cpp/embedded_shaders.cpp`, `../gpu/cpp/vulkan_loader_shim.cpp` and
`../gpu/include/photon/gpu/symbol_prefix.h`. The build is in `photon-targeting/build.gradle`
(`photongpuapriltag`). Licenses are also in `ExternalLicenses/`.
