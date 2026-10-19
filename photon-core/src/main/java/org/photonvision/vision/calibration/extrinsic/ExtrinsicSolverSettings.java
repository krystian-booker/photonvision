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

package org.photonvision.vision.calibration.extrinsic;

import io.avaje.jsonb.Json;
import java.util.List;

/**
 * Tunables for {@link ExtrinsicSolver}.
 *
 * @param useFieldLayout If true, tag poses are taken from the configured AprilTag field layout and
 *     held fixed. If false, tag poses are estimated ("self-survey") and at least one floor tag or
 *     tag with a measured height must be observed to define the floor height.
 * @param floorTagIds Tags lying flat on the floor (self-survey only)
 * @param floorTagHeightMeters Height of the floor tags' printed surface above the floor
 * @param knownTagHeights Tags (e.g. on a wall) whose center height above the floor was measured
 *     (self-survey only). Either these or floor tags define the floor height.
 * @param pixelStdDev Expected corner detection noise, pixels
 * @param odomTranslationStdDev Odometry translation noise between stops, meters
 * @param odomTranslationStdDevPerMeter Additional odometry translation noise per meter travelled
 * @param odomRotationStdDev Gyro heading noise between stops, radians
 * @param odomRotationStdDevPerRadian Additional heading noise per radian turned
 * @param estimateOdometryScale Estimate a global odometry scale factor (wheel radius error)
 * @param huberThresholdPx Corner residual (pixels) above which observations are down-weighted
 * @param maxIterations Levenberg-Marquardt iteration cap
 */
@Json
public record ExtrinsicSolverSettings(
        boolean useFieldLayout,
        List<Integer> floorTagIds,
        double floorTagHeightMeters,
        List<TagHeight> knownTagHeights,
        double pixelStdDev,
        double odomTranslationStdDev,
        double odomTranslationStdDevPerMeter,
        double odomRotationStdDev,
        double odomRotationStdDevPerRadian,
        boolean estimateOdometryScale,
        double huberThresholdPx,
        int maxIterations) {
    public static ExtrinsicSolverSettings defaults() {
        return new ExtrinsicSolverSettings(
                false,
                List.of(),
                0.0,
                List.of(),
                1.0,
                0.005,
                0.02,
                Math.toRadians(0.3),
                0.01,
                true,
                3.0,
                100);
    }

    /**
     * A tag whose center height above the floor was measured.
     *
     * @param id Fiducial ID
     * @param heightMeters Height of the tag's center above the floor
     */
    @Json
    public record TagHeight(int id, double heightMeters) {}

    public ExtrinsicSolverSettings withDatum(
            boolean useFieldLayout, List<Integer> floorTags, List<TagHeight> tagHeights) {
        return new ExtrinsicSolverSettings(
                useFieldLayout,
                floorTags,
                floorTagHeightMeters,
                tagHeights,
                pixelStdDev,
                odomTranslationStdDev,
                odomTranslationStdDevPerMeter,
                odomRotationStdDev,
                odomRotationStdDevPerRadian,
                estimateOdometryScale,
                huberThresholdPx,
                maxIterations);
    }
}
