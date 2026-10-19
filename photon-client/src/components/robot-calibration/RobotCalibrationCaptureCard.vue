<script setup lang="ts">
import { computed, ref } from "vue";
import { useTheme } from "vuetify";
import { useStateStore } from "@/stores/StateStore";
import { useRobotCalibrationStore } from "@/stores/RobotCalibrationStore";
import PvDeleteModal from "@/components/common/pv-delete-modal.vue";
import { HEADING_BUCKET_COUNT, type ExtrinsicCapturePhase } from "@/types/RobotCalibrationTypes";

const theme = useTheme();
const robotCalibrationStore = useRobotCalibrationStore();

// Number of stops recommended for a good calibration
const RECOMMENDED_STOPS = 40;

const status = computed(() => useStateStore().extrinsicCalibration);
const isRunning = computed(() => status.value.phase !== "IDLE");

const phaseDisplay = computed<{ text: string; color: string; icon: string }>(() => {
  const phase: ExtrinsicCapturePhase = status.value.phase;
  switch (phase) {
    case "WAITING_FOR_STILL":
      return { text: "Waiting for robot to stop", color: "buttonPassive", icon: "mdi-timer-sand" };
    case "COLLECTING":
      return { text: "Collecting, hold still", color: "warning", icon: "mdi-camera-burst" };
    case "WAITING_FOR_MOTION":
      return { text: "Move the robot to the next spot", color: "success", icon: "mdi-arrow-decision" };
    case "IDLE":
    default:
      return status.value.stopCount > 0
        ? { text: "Paused", color: "grey", icon: "mdi-pause" }
        : { text: "Not started", color: "grey", icon: "mdi-stop" };
  }
});

const headingCoverage = computed(() => Math.min(100, (status.value.headingsCovered / HEADING_BUCKET_COUNT) * 100));
const stopCoverage = computed(() => Math.min(100, (status.value.stopCount / RECOMMENDED_STOPS) * 100));

const uncalibratedCameras = computed(() => status.value.cameras.filter((c) => !c.hasCalibration));

const canStart = computed(
  () => useStateStore().backendConnected && robotCalibrationStore.captureSettings.cameras.length > 0
);

const busy = ref(false);
const runAction = async (action: () => Promise<boolean>) => {
  busy.value = true;
  try {
    await action();
  } finally {
    busy.value = false;
  }
};

const showResetDialog = ref(false);
</script>

<template>
  <v-card class="mb-3 rounded-12" color="surface" dark>
    <v-card-title>2. Capture</v-card-title>
    <v-card-text class="pt-0">
      <v-alert
        class="mb-3"
        color="info"
        density="compact"
        icon="mdi-information-outline"
        :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
      >
        Press Start, then drive the robot slowly near the tags. <strong>Rotate the robot in place</strong> to many
        different headings, <strong>and</strong> drive short straight legs forwards/backwards and sideways. Pause for
        about 1 second at every spot; a stop is recorded automatically each time the robot comes to rest.
        {{ RECOMMENDED_STOPS }}+ stops covering all headings are recommended.
      </v-alert>

      <div class="d-flex flex-wrap pb-3" style="gap: 8px">
        <v-btn
          v-if="!isRunning"
          color="buttonActive"
          :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
          :disabled="!canStart || busy"
          @click="runAction(() => robotCalibrationStore.startCapture())"
        >
          <v-icon start size="large">mdi-play</v-icon>
          <span>{{ status.stopCount > 0 ? "Resume" : "Start" }}</span>
        </v-btn>
        <v-btn
          v-else
          color="buttonActive"
          :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
          :disabled="busy"
          @click="runAction(() => robotCalibrationStore.pauseCapture())"
        >
          <v-icon start size="large">mdi-pause</v-icon>
          <span>Pause</span>
        </v-btn>
        <v-tooltip
          location="top"
          text="Record a stop as soon as the robot is still, even if it hasn't moved far enough"
        >
          <template #activator="{ props }">
            <v-btn
              v-bind="props"
              color="buttonPassive"
              :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
              :disabled="!isRunning || busy"
              @click="runAction(() => robotCalibrationStore.captureNow())"
            >
              <v-icon start size="large">mdi-camera</v-icon>
              <span>Capture Now</span>
            </v-btn>
          </template>
        </v-tooltip>
        <v-tooltip location="top" text="Discard the most recently recorded stop">
          <template #activator="{ props }">
            <v-btn
              v-bind="props"
              color="buttonPassive"
              :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
              :disabled="status.stopCount === 0 || busy"
              @click="runAction(() => robotCalibrationStore.removeStop(status.stopCount - 1))"
            >
              <v-icon start size="large">mdi-undo</v-icon>
              <span>Remove Last Stop</span>
            </v-btn>
          </template>
        </v-tooltip>
        <v-btn
          color="error"
          :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
          :disabled="(status.stopCount === 0 && !status.result && !isRunning) || busy"
          @click="showResetDialog = true"
        >
          <v-icon start size="large">mdi-restore</v-icon>
          <span>Reset</span>
        </v-btn>
      </div>

      <div class="d-flex flex-wrap align-center pb-3" style="gap: 8px">
        <v-chip
          label
          :color="phaseDisplay.color"
          :prepend-icon="phaseDisplay.icon"
          :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
        >
          {{ phaseDisplay.text }}
        </v-chip>
      </div>

      <v-row class="pb-3">
        <v-col cols="12" sm="6">
          <div class="d-flex justify-space-between">
            <span>Stops</span>
            <span>{{ status.stopCount }} / {{ RECOMMENDED_STOPS }}+</span>
          </div>
          <v-progress-linear
            :model-value="stopCoverage"
            :color="status.stopCount >= RECOMMENDED_STOPS ? 'success' : 'buttonPassive'"
            height="8"
            rounded
          />
        </v-col>
        <v-col cols="12" sm="6">
          <div class="d-flex justify-space-between">
            <span>Heading coverage</span>
            <span>{{ status.headingsCovered }}/{{ HEADING_BUCKET_COUNT }} headings</span>
          </div>
          <v-progress-linear
            :model-value="headingCoverage"
            :color="status.headingsCovered >= HEADING_BUCKET_COUNT ? 'success' : 'buttonPassive'"
            height="8"
            rounded
          />
        </v-col>
      </v-row>

      <v-alert
        v-if="status.message"
        class="mb-3"
        color="buttonPassive"
        density="compact"
        icon="mdi-message-text-outline"
        :text="status.message"
        :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
      />

      <v-alert
        v-if="uncalibratedCameras.length > 0"
        class="mb-3"
        color="error"
        density="compact"
        icon="mdi-alert-circle-outline"
        :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
      >
        {{ uncalibratedCameras.map((c) => `${c.nickname} (${c.resolution})`).join(", ") }}
        {{ uncalibratedCameras.length === 1 ? "has" : "have" }} no intrinsic calibration at the current resolution and
        will be ignored. Calibrate {{ uncalibratedCameras.length === 1 ? "it" : "them" }} on the Camera page first.
      </v-alert>

      <v-table v-if="status.cameras.length > 0" density="compact">
        <thead>
          <tr>
            <th>Camera</th>
            <th>Calibrated</th>
            <th>Resolution</th>
            <th>Tags Visible</th>
            <th>Stops With Tags</th>
            <th>Observations</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="camera in status.cameras" :key="camera.uniqueName">
            <td>{{ camera.nickname }}</td>
            <td>
              <v-icon :color="camera.hasCalibration ? 'success' : 'error'" size="small">
                {{ camera.hasCalibration ? "mdi-check" : "mdi-alert-circle" }}
              </v-icon>
              {{ camera.hasCalibration ? "Yes" : "Missing" }}
            </td>
            <td>{{ camera.resolution }}</td>
            <td>{{ camera.tagsVisible }}</td>
            <td>{{ camera.stopsWithTags }}</td>
            <td>{{ camera.observations }}</td>
          </tr>
        </tbody>
      </v-table>
      <p v-else class="text-medium-emphasis">Per-camera progress will be shown here once the capture is started.</p>
    </v-card-text>
  </v-card>

  <pv-delete-modal
    v-model="showResetDialog"
    :width="500"
    title="Reset Robot Calibration"
    description="Are you sure you want to discard every recorded stop and the last solve result? This action cannot be undone."
    delete-text="Reset"
    :on-confirm="() => runAction(() => robotCalibrationStore.resetCapture())"
  />
</template>

<style scoped lang="scss">
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
</style>
