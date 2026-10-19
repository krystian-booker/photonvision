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

package org.photonvision.common.dataflow.websocket;

import io.avaje.jsonb.Json;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonVersion;
import org.photonvision.common.LoadJNI;
import org.photonvision.common.LoadJNI.JNITypes;
import org.photonvision.common.configuration.NeuralNetworkModelManager;
import org.photonvision.common.configuration.PhotonConfiguration;
import org.photonvision.common.dataflow.networktables.NetworkTablesManager;
import org.photonvision.common.hardware.OsImageData;
import org.photonvision.common.hardware.Platform;
import org.photonvision.common.networking.NetworkManager;
import org.photonvision.common.networking.NetworkUtils;
import org.photonvision.vision.apriltag.AprilTagAcceleration;
import org.photonvision.vision.apriltag.GpuAprilTagDetector;
import org.photonvision.vision.pipe.impl.AprilTagDetectionPipe;
import org.photonvision.vision.processes.VisionModule;
import org.photonvision.vision.processes.VisionSourceManager;

@Json
public class UIPhotonConfiguration {
    public List<UICameraConfiguration> cameraSettings;
    public UIProgramSettings settings;

    public UIPhotonConfiguration(
            UIProgramSettings settings, List<UICameraConfiguration> cameraSettings) {
        this.cameraSettings = cameraSettings;
        this.settings = settings;
    }

    public static UIPhotonConfiguration programStateToUi(PhotonConfiguration c) {
        var ui = buildUi(c);
        var general = ui.settings.general;
        var mode = c.getHardwareSettings().aprilTagAcceleration;
        general.aprilTagAcceleration = mode.name();
        // Only probe the GPU where it would be used: creating a Vulkan device on an untested driver
        // is not free of risk
        if (AprilTagDetectionPipe.gpuRequested(mode)) {
            var gpu = GpuAprilTagDetector.queryDevice(false);
            general.gpuAprilTagAvailable = gpu.available();
            general.gpuAprilTagStatus = gpu.description();
        } else {
            general.gpuAprilTagAvailable = false;
            general.gpuAprilTagStatus =
                    mode == AprilTagAcceleration.CPU
                            ? "Disabled (CPU selected)"
                            : "Not enabled by default on this platform (select GPU to try it)";
        }
        return ui;
    }

    private static String describeGpuAcceleration(PhotonConfiguration c) {
        List<String> parts = new ArrayList<>();
        if (LoadJNI.hasLoaded(JNITypes.LIBCAMERA)) parts.add("Zerocopy Libcamera Working");
        if (AprilTagDetectionPipe.gpuRequested(c.getHardwareSettings().aprilTagAcceleration)
                && GpuAprilTagDetector.queryDevice(false).available()) {
            parts.add("Vulkan AprilTag detection");
        }
        return String.join(", ", parts);
    }

    private static UIPhotonConfiguration buildUi(PhotonConfiguration c) {
        return new UIPhotonConfiguration(
                new UIProgramSettings(
                        new UINetConfig(
                                c.getNetworkConfig(),
                                NetworkUtils.getAllActiveWiredInterfaces(),
                                NetworkManager.getInstance().networkingIsDisabled),
                        new UILightingConfig(
                                c.getHardwareSettings().ledBrightnessPercentage,
                                !c.getHardwareConfig().ledPins.isEmpty()),
                        new UIGeneralSettings(
                                PhotonVersion.versionString,
                                OsImageData.IMAGE_METADATA.isPresent()
                                        ? OsImageData.IMAGE_METADATA.get().commitTag()
                                        : "",
                                describeGpuAcceleration(c),
                                LoadJNI.hasLoaded(JNITypes.MRCAL),
                                c.getNeuralNetworkProperties().getModels(),
                                NeuralNetworkModelManager.getInstance().getSupportedBackends(),
                                c.getHardwareConfig().deviceName.isEmpty()
                                        ? Platform.getHardwareModel()
                                        : c.getHardwareConfig().deviceName,
                                Platform.getPlatformName(),
                                Platform.getNativePlatform(),
                                NetworkTablesManager.getInstance().conflictingHostname,
                                NetworkTablesManager.getInstance().conflictingCameras),
                        c.getFieldLayout()),
                VisionSourceManager.getInstance().getVisionModules().stream()
                        .map(VisionModule::toUICameraConfig)
                        .toList());
    }
}
