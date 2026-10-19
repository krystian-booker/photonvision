/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package org.photonvision.vision.apriltag;

/** Where AprilTag detection runs. */
public enum AprilTagAcceleration {
    /**
     * Use the GPU on platforms where it has been validated (currently the Rubik Pi 3 / QCS6490),
     * otherwise the CPU. Always falls back to the CPU if the GPU can't be used.
     */
    AUTO,
    /** Use any hardware Vulkan GPU, falling back to the CPU if it can't be used. */
    GPU,
    /** Always use the CPU detector. */
    CPU
}
