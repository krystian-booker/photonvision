<script setup lang="ts">
import { computed, ref, watch } from "vue";
import { useTheme } from "vuetify";
import { useCameraSettingsStore } from "@/stores/settings/CameraSettingsStore";
import { useStateStore } from "@/stores/StateStore";
import { getRequestErrorMessage } from "@/stores/RobotCalibrationStore";
import PvNumberInput from "@/components/common/pv-number-input.vue";
import PvDeleteModal from "@/components/common/pv-delete-modal.vue";
import TooltippedLabel from "@/components/common/pv-tooltipped-label.vue";
import { formatStdDev, formatTransformComponent, transformComponents } from "@/lib/RobotCalibrationUtils";
import type { RobotToCameraTransform } from "@/types/SettingTypes";

const theme = useTheme();

const stored = computed(() => useCameraSettingsStore().currentCameraSettings.robotToCamera);

const componentTooltips: Record<keyof RobotToCameraTransform, string> = {
  x: "Distance of the camera forward of the robot center, in meters",
  y: "Distance of the camera left of the robot center, in meters",
  z: "Height of the camera above the floor (robot origin), in meters",
  rollDeg: "Rotation of the camera about the robot's X (forward) axis, in degrees",
  pitchDeg: "Rotation of the camera about the robot's Y (left) axis, in degrees (WPILib convention: negative tilts up)",
  yawDeg: "Rotation of the camera about the robot's Z (up) axis, in degrees (positive turns left)"
};

const zeroTransform = (): RobotToCameraTransform => ({ x: 0, y: 0, z: 0, rollDeg: 0, pitchDeg: 0, yawDeg: 0 });

const form = ref<RobotToCameraTransform>(zeroTransform());
const resetForm = () => {
  const s = stored.value;
  form.value = s
    ? { x: s.x, y: s.y, z: s.z, rollDeg: s.rollDeg, pitchDeg: s.pitchDeg, yawDeg: s.yawDeg }
    : zeroTransform();
};
// Only reset the form when the camera or the stored values actually change, so websocket updates don't discard edits
watch(
  () => [useCameraSettingsStore().currentCameraSettings.uniqueName, JSON.stringify(stored.value ?? null)],
  resetForm,
  { immediate: true }
);

const formValid = computed(() => transformComponents.every((c) => Number.isFinite(form.value[c.key])));
const isDirty = computed(() => {
  const s = stored.value ?? zeroTransform();
  return !stored.value || transformComponents.some((c) => form.value[c.key] !== s[c.key]);
});

const saving = ref(false);
const saveRobotToCamera = async (transform: RobotToCameraTransform | null) => {
  saving.value = true;
  try {
    await useCameraSettingsStore().setRobotToCamera(transform);
    useStateStore().showSnackbarMessage({
      color: "success",
      message: transform ? "Saved the robot-to-camera transform" : "Cleared the robot-to-camera transform"
    });
  } catch (error) {
    useStateStore().showSnackbarMessage({
      color: "error",
      message: getRequestErrorMessage(
        error,
        transform ? "save the robot-to-camera transform" : "clear the robot-to-camera transform"
      )
    });
  } finally {
    saving.value = false;
  }
};

const showClearDialog = ref(false);
</script>

<template>
  <v-card class="mb-3 rounded-12" color="surface" dark>
    <v-card-title>Robot to Camera Transform</v-card-title>
    <v-card-text class="pt-0">
      <p class="pb-3">
        Where this camera is mounted on the robot, used to turn camera measurements into robot poses. Measure it by hand
        and enter it below, or use <router-link to="/robotCalibration">Robot Calibration</router-link> to measure it
        automatically.
      </p>

      <template v-if="stored">
        <div class="d-flex flex-wrap align-center pb-2" style="gap: 8px">
          <v-chip
            label
            :color="stored.source === 'calibrated' ? 'success' : 'buttonPassive'"
            :prepend-icon="stored.source === 'calibrated' ? 'mdi-axis-arrow' : 'mdi-pencil-ruler'"
            :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
          >
            {{ stored.source === "calibrated" ? "Calibrated" : "Manual" }}
          </v-chip>
          <span class="text-medium-emphasis">
            Updated {{ new Date(stored.updatedAtMillis).toLocaleString() }}
            <template v-if="stored.source === 'calibrated' && stored.rmsReprojectionErrorPx > 0">
              &middot; RMS {{ stored.rmsReprojectionErrorPx.toFixed(2) }} px
            </template>
          </span>
        </div>
        <v-table density="compact" class="pb-3">
          <thead>
            <tr>
              <th />
              <th v-for="component in transformComponents" :key="component.key">{{ component.label }}</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>Saved</td>
              <td v-for="component in transformComponents" :key="component.key">
                {{ formatTransformComponent(stored[component.key], component.isAngle) }}
              </td>
            </tr>
            <tr v-if="stored.stdDevs && stored.stdDevs.length >= transformComponents.length">
              <td>±1σ</td>
              <td v-for="(component, index) in transformComponents" :key="component.key">
                {{ formatStdDev(stored.stdDevs[index], component.isAngle) }}
              </td>
            </tr>
          </tbody>
        </v-table>
      </template>
      <v-alert
        v-else
        class="mb-3"
        color="primary"
        density="compact"
        icon="mdi-alert-circle-outline"
        text="No robot-to-camera transform is saved for this camera."
        :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
      />

      <div class="pb-1">
        <tooltipped-label
          label="Edit Transform"
          tooltip="WPILib robot coordinates: X forward, Y left, Z up, measured from the robot origin"
        />
      </div>
      <v-row>
        <v-col cols="12" sm="6" class="pt-0 pb-0">
          <pv-number-input
            v-for="component in transformComponents.filter((c) => !c.isAngle)"
            :key="component.key"
            v-model="form[component.key]"
            :label="`${component.label} (m)`"
            :tooltip="componentTooltips[component.key]"
            :label-cols="5"
            :step="0.001"
          />
        </v-col>
        <v-col cols="12" sm="6" class="pt-0 pb-0">
          <pv-number-input
            v-for="component in transformComponents.filter((c) => c.isAngle)"
            :key="component.key"
            v-model="form[component.key]"
            :label="`${component.label} (°)`"
            :tooltip="componentTooltips[component.key]"
            :label-cols="5"
            :step="0.1"
          />
        </v-col>
      </v-row>

      <div class="d-flex flex-wrap pt-3" style="gap: 8px">
        <v-btn
          color="buttonActive"
          :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
          :loading="saving"
          :disabled="!formValid || !isDirty || useCameraSettingsStore().needsCameraConfiguration"
          @click="saveRobotToCamera({ ...form })"
        >
          <v-icon start size="large">mdi-content-save</v-icon>
          <span>Save</span>
        </v-btn>
        <v-btn
          color="buttonPassive"
          :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
          :disabled="!isDirty || !stored"
          @click="resetForm"
        >
          <v-icon start size="large">mdi-undo</v-icon>
          <span>Revert</span>
        </v-btn>
        <v-btn
          color="error"
          :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
          :disabled="!stored || saving"
          @click="showClearDialog = true"
        >
          <v-icon start size="large">mdi-delete-outline</v-icon>
          <span>Clear</span>
        </v-btn>
        <v-spacer />
        <v-btn
          color="buttonPassive"
          :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
          to="/robotCalibration"
        >
          <v-icon start size="large">mdi-axis-arrow</v-icon>
          <span>Robot Calibration</span>
        </v-btn>
      </div>
    </v-card-text>
  </v-card>

  <pv-delete-modal
    v-model="showClearDialog"
    :width="500"
    title="Clear Robot to Camera Transform"
    :description="`Are you sure you want to clear the robot-to-camera transform of '${useCameraSettingsStore().currentCameraName}'?`"
    delete-text="Clear"
    :on-confirm="() => saveRobotToCamera(null)"
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

  td {
    font-family: monospace;
    white-space: nowrap;
  }

  td:first-child {
    font-family: inherit;
    text-align: left;
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
