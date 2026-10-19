<script setup lang="ts">
import { computed, ref, watch } from "vue";
import { useTheme } from "vuetify";
import { useStateStore } from "@/stores/StateStore";
import { useCameraSettingsStore } from "@/stores/settings/CameraSettingsStore";
import { useRobotCalibrationStore } from "@/stores/RobotCalibrationStore";
import PvNumberInput from "@/components/common/pv-number-input.vue";
import PvSelect from "@/components/common/pv-select.vue";
import TooltippedLabel from "@/components/common/pv-tooltipped-label.vue";
import { PipelineType } from "@/types/PipelineTypes";
import { PlaceholderCameraSettings, type UiCameraConfiguration } from "@/types/SettingTypes";
import { getResolutionString, resolutionsAreEqual } from "@/lib/PhotonUtils";
import { formatDegrees, formatMeters } from "@/lib/RobotCalibrationUtils";
import { length } from "@adam-rocska/units-and-measurement/length";

const theme = useTheme();
const robotCalibrationStore = useRobotCalibrationStore();

const status = computed(() => useStateStore().extrinsicCalibration);
// A session exists once the capture has been started or a stop has been recorded
const sessionActive = computed(() => status.value.phase !== "IDLE" || status.value.stopCount > 0);

interface CameraReadiness {
  uniqueName: string;
  nickname: string;
  isAprilTag: boolean;
  resolution: string;
  isCalibrated: boolean;
}

const availableCameras = computed<UiCameraConfiguration[]>(() =>
  Object.values(useCameraSettingsStore().cameras).filter((c) => c.uniqueName !== PlaceholderCameraSettings.uniqueName)
);

const cameraReadiness = computed<CameraReadiness[]>(() =>
  availableCameras.value.map((camera) => {
    const videoFormat = camera.validVideoFormats[camera.pipelineSettings.cameraVideoModeIndex];
    return {
      uniqueName: camera.uniqueName,
      nickname: camera.nickname,
      isAprilTag: camera.pipelineSettings.pipelineType === PipelineType.AprilTag,
      resolution: videoFormat ? getResolutionString(videoFormat.resolution) : "Unknown",
      isCalibrated:
        videoFormat !== undefined &&
        camera.completeCalibrations.some((c) => resolutionsAreEqual(c.resolution, videoFormat.resolution))
    };
  })
);

// Select every camera until the user picks cameras themselves
watch(
  () => availableCameras.value.map((c) => c.uniqueName),
  (names) => {
    if (!robotCalibrationStore.cameraSelectionTouched) {
      robotCalibrationStore.captureSettings.cameras = [...names];
    }
  },
  { immediate: true }
);

const selectedCameras = computed<string[]>({
  get: () => robotCalibrationStore.captureSettings.cameras,
  set: (v) => {
    robotCalibrationStore.cameraSelectionTouched = true;
    robotCalibrationStore.captureSettings.cameras = v;
  }
});

// Show the settings of an existing session, which the backend keeps when the capture is paused or resumed
watch(
  () => JSON.stringify(status.value.settings),
  () => {
    if (!sessionActive.value) return;
    const settings = status.value.settings;
    robotCalibrationStore.captureSettings = { ...settings, cameras: [...settings.cameras] };
    robotCalibrationStore.cameraSelectionTouched = true;
  },
  { immediate: true }
);

const tagSizeUnit = ref<"in" | "mm">("in");
const tagSize = computed<number>({
  get: () =>
    parseFloat(length.m(robotCalibrationStore.captureSettings.tagSizeMeters)[tagSizeUnit.value].value.toFixed(3)),
  set: (v) => {
    if (Number.isFinite(v) && v > 0) {
      robotCalibrationStore.captureSettings.tagSizeMeters = length[tagSizeUnit.value](v).m.value;
    }
  }
});
const tagSizeInches = computed(() => length.m(robotCalibrationStore.captureSettings.tagSizeMeters).in.value);
</script>

<template>
  <v-card class="mb-3 rounded-12" color="surface" dark>
    <v-card-title>1. Setup</v-card-title>
    <v-card-text class="pt-0">
      <p class="pb-2">
        Robot calibration finds where each camera is mounted on the robot (the robot-to-camera transform) by driving the
        robot around a few AprilTags while PhotonVision records what every camera sees each time the robot stops.
      </p>
      <ul class="pl-5 pb-3">
        <li>
          Your robot code must call PhotonLib's <code>RobotCalibrationPublisher.update(odometryPose)</code> every loop
          with an <strong>odometry-only</strong> pose (do not fuse vision measurements into it).
        </li>
        <li>
          Every camera needs an AprilTag pipeline and an intrinsic calibration at the resolution it is currently running
          at (see the Camera page).
        </li>
        <li>
          Place several AprilTags around a roughly 3 x 3 m area, facing the robot. Unless you use the field layout, at
          least one tag must lie flat on the floor (face up) or have its center height tape-measured.
        </li>
      </ul>

      <v-alert
        v-if="!useStateStore().backendConnected"
        class="mb-3"
        color="error"
        density="compact"
        icon="mdi-server-network-off"
        text="Not connected to the PhotonVision backend. The values below may be out of date."
        :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
      />

      <v-card-subtitle class="pl-0 pb-2 opacity-100">Robot Connection</v-card-subtitle>
      <div class="d-flex flex-wrap align-center pb-3" style="gap: 8px">
        <v-chip
          label
          :color="status.odometryConnected ? 'success' : 'error'"
          :prepend-icon="status.odometryConnected ? 'mdi-robot' : 'mdi-robot-off'"
          :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
        >
          {{ status.odometryConnected ? "Receiving robot odometry" : "No odometry from the robot" }}
        </v-chip>
        <v-chip
          v-if="status.odometryConnected"
          label
          :color="status.robotStill ? 'success' : 'buttonPassive'"
          :prepend-icon="status.robotStill ? 'mdi-pause-circle-outline' : 'mdi-run-fast'"
          :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
        >
          {{ status.robotStill ? "Robot still" : "Robot moving" }}
        </v-chip>
        <span v-if="status.odometry.length >= 3" class="odometry-readout">
          x {{ formatMeters(status.odometry[0]) }}, y {{ formatMeters(status.odometry[1]) }}, heading
          {{ formatDegrees(status.odometry[2]) }}
        </span>
      </div>
      <v-alert
        v-if="!status.odometryConnected"
        class="mb-3"
        color="warning"
        density="compact"
        icon="mdi-alert-outline"
        text="Make sure the robot is on, connected to NetworkTables and calling RobotCalibrationPublisher.update(odometryPose) every loop."
        :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
      />

      <v-card-subtitle class="pl-0 pb-2 opacity-100">Cameras</v-card-subtitle>
      <div v-if="cameraReadiness.length === 0" class="pb-3">
        No cameras are activated. Head to the <router-link to="/cameraConfigs">Camera Matching</router-link> page to set
        some up.
      </div>
      <v-table v-else density="compact" class="pb-3">
        <thead>
          <tr>
            <th>Use</th>
            <th>Camera</th>
            <th>Pipeline</th>
            <th>Resolution</th>
            <th>Calibrated</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="camera in cameraReadiness" :key="camera.uniqueName">
            <td>
              <v-checkbox-btn
                v-model="selectedCameras"
                :value="camera.uniqueName"
                color="primary"
                density="compact"
                :disabled="sessionActive"
              />
            </td>
            <td>{{ camera.nickname }}</td>
            <td>
              <v-icon :color="camera.isAprilTag ? 'success' : 'error'" size="small">
                {{ camera.isAprilTag ? "mdi-check" : "mdi-close" }}
              </v-icon>
              {{ camera.isAprilTag ? "AprilTag" : "Not AprilTag" }}
            </td>
            <td>{{ camera.resolution }}</td>
            <td>
              <v-icon :color="camera.isCalibrated ? 'success' : 'error'" size="small">
                {{ camera.isCalibrated ? "mdi-check" : "mdi-close" }}
              </v-icon>
              {{ camera.isCalibrated ? "Yes" : "No" }}
            </td>
          </tr>
        </tbody>
      </v-table>
      <v-alert
        v-if="cameraReadiness.some((c) => selectedCameras.includes(c.uniqueName) && (!c.isAprilTag || !c.isCalibrated))"
        class="mb-3"
        color="error"
        density="compact"
        icon="mdi-alert-circle-outline"
        :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
      >
        Some selected cameras are not ready. Each camera must run an AprilTag pipeline and be calibrated at its current
        resolution, otherwise it can't contribute to the calibration.
      </v-alert>
      <v-alert
        v-if="sessionActive"
        class="mb-3"
        color="info"
        density="compact"
        icon="mdi-information-outline"
        text="A capture session is in progress. Reset the session to change the cameras or tag size."
        :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
      />

      <v-card-subtitle class="pl-0 pb-0 opacity-100">Tags</v-card-subtitle>
      <pv-select
        v-model="tagSizeUnit"
        label="Tag Size Unit"
        tooltip="Units used for the tag size input"
        :select-cols="8"
        :items="[
          { value: 'in', name: 'Inches' },
          { value: 'mm', name: 'Millimeters' }
        ]"
      />
      <pv-number-input
        v-model="tagSize"
        :label="`Tag Size (${tagSizeUnit})`"
        tooltip="Side length of the black square of the AprilTags (not including the white border). FRC tags are 6.5 in (165.1 mm)."
        :label-cols="4"
        :step="tagSizeUnit === 'mm' ? 0.1 : 0.01"
        :disabled="sessionActive"
        :rules="[(v) => v > 0 || 'Size must be positive']"
      />
      <p class="pb-3 text-medium-emphasis">
        {{ robotCalibrationStore.captureSettings.tagSizeMeters.toFixed(4) }} m ({{ tagSizeInches.toFixed(2) }} in)
      </p>

      <v-expansion-panels>
        <v-expansion-panel bg-color="surface">
          <v-expansion-panel-title>
            <tooltipped-label
              label="Advanced Capture Settings"
              tooltip="Controls when PhotonVision records a stop. The defaults work for most robots."
            />
          </v-expansion-panel-title>
          <v-expansion-panel-text>
            <pv-number-input
              v-model="robotCalibrationStore.captureSettings.minTranslationMeters"
              label="Min Translation (m)"
              tooltip="The robot must move at least this far (or rotate at least the minimum rotation) since the last stop before a new stop is recorded"
              :label-cols="6"
              :step="0.01"
              :rules="[(v) => v >= 0 || 'Must not be negative']"
            />
            <pv-number-input
              v-model="robotCalibrationStore.captureSettings.minRotationDegrees"
              label="Min Rotation (°)"
              tooltip="The robot must rotate at least this much (or move at least the minimum translation) since the last stop before a new stop is recorded"
              :label-cols="6"
              :rules="[(v) => v >= 0 || 'Must not be negative']"
            />
            <pv-number-input
              v-model="robotCalibrationStore.captureSettings.settleSeconds"
              label="Settle Time (s)"
              tooltip="How long the robot must be still before frames are collected"
              :label-cols="6"
              :step="0.1"
              :rules="[(v) => v >= 0 || 'Must not be negative']"
            />
            <pv-number-input
              v-model="robotCalibrationStore.captureSettings.framesPerStop"
              label="Frames per Stop"
              tooltip="Number of frames averaged from each camera at every stop"
              :label-cols="6"
              :rules="[(v) => (v >= 1 && Number.isInteger(v)) || 'Must be a positive whole number']"
            />
          </v-expansion-panel-text>
        </v-expansion-panel>
      </v-expansion-panels>
    </v-card-text>
  </v-card>
</template>

<style scoped lang="scss">
.odometry-readout {
  font-family: monospace;
}

.v-table {
  width: 100%;
  text-align: center;

  th {
    text-align: center !important;
  }

  th,
  td {
    font-size: 1rem !important;
  }
}

a:link,
a:visited {
  color: rgb(var(--v-theme-buttonActive));
  text-decoration: none;
}
a:hover {
  text-decoration: underline;
}
</style>
