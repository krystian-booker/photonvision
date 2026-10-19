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

package org.photonvision.benchmark;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.common.LoadJNI;
import org.photonvision.common.util.TestUtils;
import org.photonvision.jni.LibraryLoader;
import org.photonvision.vision.apriltag.AprilTagAcceleration;

public class AprilTagBenchmarkTest {
    @BeforeAll
    public static void init() throws IOException {
        LoadJNI.loadLibraries();
        LibraryLoader.loadTargeting();
    }

    @Test
    public void parsesScenarios() {
        var s = AprilTagBenchmark.parseScenarios("baseline, gpu-d2,cpu:d3:t2:color");
        assertEquals(3, s.size());
        assertEquals(1, s.get(0).decimate());
        assertEquals(4, s.get(0).threads());
        assertEquals(AprilTagAcceleration.GPU, s.get(1).acceleration());
        assertEquals(3, s.get(2).decimate());
        assertEquals(2, s.get(2).threads());
        assertTrue(!s.get(2).grayDecode());
        assertThrows(IllegalArgumentException.class, () -> AprilTagBenchmark.parseScenarios("fast"));
    }

    @Test
    public void runsOnTestImages() throws Exception {
        var images = TestUtils.getResourcesFolderPath(false).resolve("testimages").resolve("2024");
        var out = Files.createTempFile("apriltag-benchmark", ".json");
        var report =
                AprilTagBenchmark.runAll(
                        images, List.of(1, 2), AprilTagBenchmark.parseScenarios("cpu-d2,gpu-d2"), 1.0, out);
        assertEquals(4, report.runs().size());
        for (var r : report.runs()) {
            assertTrue(r.totalFps() > 0, r.toString());
            assertTrue(r.tagsPerFrame() > 0, r.toString());
        }
        assertTrue(Files.readString(out).contains("\"runs\""));
        Files.deleteIfExists(out);
    }
}
