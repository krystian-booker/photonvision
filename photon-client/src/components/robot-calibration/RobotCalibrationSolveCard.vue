<script setup lang="ts">
import { computed, inject, ref, useTemplateRef, watch } from "vue";
import { useTheme } from "vuetify";
import { useStateStore } from "@/stores/StateStore";
import { useSettingsStore } from "@/stores/settings/GeneralSettingsStore";
import { useCameraSettingsStore } from "@/stores/settings/CameraSettingsStore";
import { useRobotCalibrationStore } from "@/stores/RobotCalibrationStore";
import PvNumberInput from "@/components/common/pv-number-input.vue";
import PvSwitch from "@/components/common/pv-switch.vue";
import TooltippedLabel from "@/components/common/pv-tooltipped-label.vue";
import { parseJsonFile } from "@/lib/PhotonUtils";
import {
  formatDifference,
  formatMeters,
  formatDegrees,
  formatStdDev,
  formatTransformComponent,
  transformComponents,
  transformDifference,
  type TransformComponent
} from "@/lib/RobotCalibrationUtils";
import { DefaultExtrinsicSolverSettings, type ExtrinsicCameraResult } from "@/types/RobotCalibrationTypes";
import type { StoredRobotToCamera } from "@/types/SettingTypes";

const theme = useTheme();
const robotCalibrationStore = useRobotCalibrationStore();
const backendHost = inject<string>("backendHost");

const status = computed(() => useStateStore().extrinsicCalibration);
const result = computed(() => status.value.result);

// Datum
const datumMode = computed<"field" | "survey">({
  get: () => (robotCalibrationStore.solverSettings.useFieldLayout ? "field" : "survey"),
  set: (v) => (robotCalibrationStore.solverSettings.useFieldLayout = v === "field")
});
const fieldTagCount = computed(() => useSettingsStore().currentFieldLayout["field-tags"]?.length ?? 0);

const parseTagIds = (text: string): number[] | undefined => {
  const ids = text
    .split(/[\s,]+/)
    .filter((token) => token.length > 0)
    .map(Number);
  return ids.every((id) => Number.isInteger(id) && id >= 0) ? [...new Set(ids)] : undefined;
};
const floorTagIdsText = ref(robotCalibrationStore.solverSettings.floorTagIds.join(", "));
const floorTagIdsValid = computed(() => parseTagIds(floorTagIdsText.value) !== undefined);
watch(floorTagIdsText, (text) => {
  const ids = parseTagIds(text);
  if (ids !== undefined) robotCalibrationStore.solverSettings.floorTagIds = ids;
});

const addKnownTagHeight = () => {
  robotCalibrationStore.solverSettings.knownTagHeights.push({ id: 0, heightMeters: 0 });
};
const removeKnownTagHeight = (index: number) => {
  robotCalibrationStore.solverSettings.knownTagHeights.splice(index, 1);
};
const knownTagHeightsValid = computed(() =>
  robotCalibrationStore.solverSettings.knownTagHeights.every(
    (h) => Number.isInteger(h.id) && h.id >= 0 && Number.isFinite(h.heightMeters)
  )
);

const selfSurvey = computed(() => !robotCalibrationStore.solverSettings.useFieldLayout);
const datumValid = computed(
  () =>
    !selfSurvey.value ||
    robotCalibrationStore.solverSettings.floorTagIds.length > 0 ||
    robotCalibrationStore.solverSettings.knownTagHeights.length > 0
);
const solverSettingsValid = computed(
  () => datumValid.value && (!selfSurvey.value || (floorTagIdsValid.value && knownTagHeightsValid.value))
);

const restoreDefaultNoiseSettings = () => {
  const { useFieldLayout, floorTagIds, floorTagHeightMeters, knownTagHeights } = robotCalibrationStore.solverSettings;
  robotCalibrationStore.solverSettings = {
    ...DefaultExtrinsicSolverSettings,
    useFieldLayout,
    floorTagIds,
    floorTagHeightMeters,
    knownTagHeights
  };
};

// Results
interface CameraComparison {
  result: ExtrinsicCameraResult;
  current?: StoredRobotToCamera;
  difference?: Record<TransformComponent, number>;
}
const cameraComparisons = computed<CameraComparison[]>(() =>
  (result.value?.cameras ?? []).map((camera) => {
    const current = useCameraSettingsStore().cameras[camera.uniqueName]?.robotToCamera;
    return {
      result: camera,
      current: current,
      difference: current ? transformDifference(camera, current) : undefined
    };
  })
);

// Cameras to save, defaulting to every camera whenever a new result arrives
const camerasToSave = ref<string[]>([]);
watch(
  () => JSON.stringify(result.value ?? null),
  () => {
    camerasToSave.value = result.value?.success ? result.value.cameras.map((c) => c.uniqueName) : [];
  },
  { immediate: true }
);

const saveToCameras = async () => {
  await robotCalibrationStore.commit(camerasToSave.value);
};

// Dataset import/export
const exportDatasetURL = computed(() => `http://${backendHost}/api/extrinsics/dataset`);
const exportDataset = useTemplateRef("exportDataset");
const openExportDatasetPrompt = () => {
  exportDataset.value?.click();
};

const importDatasetFile = useTemplateRef("importDatasetFile");
const openImportDatasetPrompt = () => {
  importDatasetFile.value?.click();
};
const importDataset = async () => {
  const input = importDatasetFile.value;
  const files = input?.files;
  if (!input || !files?.length) return;

  try {
    const data = await parseJsonFile<Record<string, unknown>>(files[0]);
    await robotCalibrationStore.importDataset(data);
  } catch {
    useStateStore().showSnackbarMessage({
      color: "error",
      message: "The selected file is not a valid robot calibration dataset."
    });
  } finally {
    // Allow importing the same file again
    input.value = "";
  }
};

const formatDate = (millis: number) => new Date(millis).toLocaleString();
</script>

<template>
  <v-card class="mb-3 rounded-12" color="surface" dark>
    <v-card-title class="pb-2">
      <div class="d-flex flex-wrap">
        <v-col cols="12" md="6" class="pa-0">
          <v-card-title class="pa-0">3. Solve</v-card-title>
        </v-col>
        <v-col cols="6" md="3" class="d-flex align-center pt-0 pb-0 pl-0">
          <v-btn
            color="buttonPassive"
            style="width: 100%"
            :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
            :disabled="!useStateStore().backendConnected"
            @click="openImportDatasetPrompt"
          >
            <v-icon start size="large">mdi-import</v-icon>
            <span>Import Data</span>
          </v-btn>
          <input ref="importDatasetFile" type="file" accept=".json" style="display: none" @change="importDataset" />
        </v-col>
        <v-col cols="6" md="3" class="d-flex align-center pt-0 pb-0 pr-0">
          <v-btn
            color="buttonPassive"
            style="width: 100%"
            :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
            :disabled="status.stopCount === 0"
            @click="openExportDatasetPrompt"
          >
            <v-icon start size="large">mdi-export</v-icon>
            <span>Export Data</span>
          </v-btn>
          <a
            ref="exportDataset"
            style="color: black; text-decoration: none; display: none"
            :href="exportDatasetURL"
            download="robot-calibration-dataset.json"
            target="_blank"
          />
        </v-col>
      </div>
    </v-card-title>
    <v-card-text class="pt-0">
      <v-card-subtitle class="pl-0 pb-2 opacity-100">Tag Positions</v-card-subtitle>
      <v-btn-toggle v-model="datumMode" mandatory class="fill w-100 mb-3">
        <v-btn
          value="field"
          color="buttonPassive"
          class="w-50"
          :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
        >
          <v-icon start size="large">mdi-soccer-field</v-icon>
          <span>Field Layout</span>
        </v-btn>
        <v-btn
          value="survey"
          color="buttonPassive"
          class="w-50"
          :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
        >
          <v-icon start size="large">mdi-map-marker-radius</v-icon>
          <span>Self-Survey</span>
        </v-btn>
      </v-btn-toggle>

      <template v-if="!selfSurvey">
        <p class="pb-3">
          The tags are fixed at their positions in the AprilTag field layout configured on the Settings page (currently
          {{ fieldTagCount }} tags). Only use this mode if your tags are mounted exactly where the layout says they are.
        </p>
      </template>
      <template v-else>
        <p class="pb-2">
          The tag positions are estimated together with the cameras, so the tags can be placed anywhere. To pin down
          height and tilt, the solver needs at least <strong>one tag lying flat on the floor</strong> (face up) or
          <strong>one tag with a measured center height</strong>.
        </p>
        <div class="d-flex">
          <v-col cols="4" class="d-flex align-center pl-0">
            <tooltipped-label
              label="Floor Tag IDs"
              tooltip="IDs of tags lying flat on the floor, face up, separated by commas"
            />
          </v-col>
          <v-col cols="8" class="pr-0">
            <v-text-field
              v-model="floorTagIdsText"
              density="compact"
              color="primary"
              variant="underlined"
              placeholder="e.g. 1, 4, 7"
              hide-details="auto"
              :rules="[
                (v: string) => parseTagIds(v) !== undefined || 'Enter tag IDs separated by commas, e.g. 1, 4, 7'
              ]"
            />
          </v-col>
        </div>
        <pv-number-input
          v-model="robotCalibrationStore.solverSettings.floorTagHeightMeters"
          label="Floor Tag Height (m)"
          tooltip="Height of the center of the floor tags above the floor, e.g. if they are mounted on a board"
          :label-cols="4"
          :step="0.001"
        />
        <div class="d-flex align-center pt-2">
          <tooltipped-label
            label="Known Tag Heights"
            tooltip="Tags with a tape-measured height of their center above the floor, in meters"
          />
          <v-spacer />
          <v-btn
            size="small"
            color="buttonPassive"
            :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
            @click="addKnownTagHeight"
          >
            <v-icon start>mdi-plus</v-icon>
            <span>Add Tag</span>
          </v-btn>
        </div>
        <div
          v-for="(knownHeight, index) in robotCalibrationStore.solverSettings.knownTagHeights"
          :key="index"
          class="d-flex align-center"
          style="gap: 16px"
        >
          <v-text-field
            v-model.number="knownHeight.id"
            label="Tag ID"
            type="number"
            density="compact"
            color="primary"
            variant="underlined"
            hide-details="auto"
            :rules="[(v: number) => (Number.isInteger(v) && v >= 0) || 'Invalid tag ID']"
          />
          <v-text-field
            v-model.number="knownHeight.heightMeters"
            label="Center Height (m)"
            type="number"
            step="0.001"
            density="compact"
            color="primary"
            variant="underlined"
            hide-details="auto"
            :rules="[(v: number) => Number.isFinite(v) || 'Invalid height']"
          />
          <v-btn icon size="small" variant="text" @click="removeKnownTagHeight(index)">
            <v-icon color="error">mdi-delete</v-icon>
          </v-btn>
        </div>
        <v-alert
          v-if="!datumValid"
          class="mt-3"
          color="warning"
          density="compact"
          icon="mdi-alert-outline"
          text="Add at least one floor tag ID or one known tag height before solving."
          :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
        />
      </template>

      <v-expansion-panels class="mt-3">
        <v-expansion-panel bg-color="surface">
          <v-expansion-panel-title>
            <tooltipped-label
              label="Advanced Solver Settings"
              tooltip="Noise model used by the solver. The defaults work for most robots."
            />
          </v-expansion-panel-title>
          <v-expansion-panel-text>
            <pv-number-input
              v-model="robotCalibrationStore.solverSettings.pixelStdDev"
              label="Pixel Noise (px)"
              tooltip="Expected standard deviation of the tag corner detections"
              :label-cols="7"
              :step="0.1"
            />
            <pv-number-input
              v-model="robotCalibrationStore.solverSettings.odomTranslationStdDev"
              label="Odometry Translation Noise (m)"
              tooltip="Fixed standard deviation of the odometry translation between stops"
              :label-cols="7"
              :step="0.001"
            />
            <pv-number-input
              v-model="robotCalibrationStore.solverSettings.odomTranslationStdDevPerMeter"
              label="Odometry Translation Noise per Meter"
              tooltip="Additional odometry translation standard deviation per meter driven between stops"
              :label-cols="7"
              :step="0.001"
            />
            <pv-number-input
              v-model="robotCalibrationStore.solverSettings.odomRotationStdDev"
              label="Odometry Rotation Noise (rad)"
              tooltip="Fixed standard deviation of the odometry rotation between stops"
              :label-cols="7"
              :step="0.001"
            />
            <pv-number-input
              v-model="robotCalibrationStore.solverSettings.odomRotationStdDevPerRadian"
              label="Odometry Rotation Noise per Radian"
              tooltip="Additional odometry rotation standard deviation per radian turned between stops"
              :label-cols="7"
              :step="0.001"
            />
            <pv-switch
              v-model="robotCalibrationStore.solverSettings.estimateOdometryScale"
              label="Estimate Odometry Scale"
              tooltip="Estimate a scale correction for the odometry, e.g. to compensate for an incorrect wheel diameter"
              :label-cols="7"
              :switch-cols="5"
            />
            <pv-number-input
              v-model="robotCalibrationStore.solverSettings.huberThresholdPx"
              label="Outlier Threshold (px)"
              tooltip="Reprojection errors larger than this are down-weighted as outliers (Huber loss)"
              :label-cols="7"
              :step="0.1"
            />
            <pv-number-input
              v-model="robotCalibrationStore.solverSettings.maxIterations"
              label="Max Iterations"
              tooltip="Maximum number of solver iterations"
              :label-cols="7"
            />
            <v-btn
              size="small"
              color="buttonPassive"
              class="mt-2"
              :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
              @click="restoreDefaultNoiseSettings"
            >
              <v-icon start>mdi-restore</v-icon>
              <span>Restore Defaults</span>
            </v-btn>
          </v-expansion-panel-text>
        </v-expansion-panel>
      </v-expansion-panels>

      <v-btn
        class="mt-4"
        block
        color="buttonActive"
        :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
        :loading="robotCalibrationStore.solving"
        :disabled="!useStateStore().backendConnected || status.stopCount === 0 || !solverSettingsValid"
        @click="robotCalibrationStore.solve()"
      >
        <v-icon start size="large">mdi-calculator-variant-outline</v-icon>
        <span>Solve</span>
      </v-btn>
      <p v-if="status.stopCount === 0" class="pt-2 text-medium-emphasis">
        Capture some stops (or import a dataset) before solving.
      </p>
      <p v-else-if="robotCalibrationStore.solving" class="pt-2 text-medium-emphasis">
        Solving, this can take a few seconds...
      </p>

      <template v-if="result">
        <v-divider class="mt-4 mb-4" />
        <v-card-subtitle class="pl-0 pb-2 opacity-100">Results</v-card-subtitle>
        <v-alert
          class="mb-3"
          density="compact"
          :color="result.success ? 'success' : 'error'"
          :icon="result.success ? 'mdi-check-circle-outline' : 'mdi-close-circle-outline'"
          :text="result.message || (result.success ? 'Solve succeeded' : 'Solve failed')"
          :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
        />
        <v-table density="compact" class="mb-3 summary-table">
          <tbody>
            <tr>
              <td>RMS Reprojection Error</td>
              <td>
                {{ Number.isFinite(result.rmsReprojectionErrorPx) ? result.rmsReprojectionErrorPx.toFixed(2) : "-" }} px
              </td>
            </tr>
            <tr>
              <td>Odometry Scale</td>
              <td>{{ Number.isFinite(result.odometryScale) ? result.odometryScale.toFixed(4) : "-" }}</td>
            </tr>
            <tr>
              <td>Stops Used</td>
              <td>{{ result.stopsUsed }}</td>
            </tr>
            <tr>
              <td>Iterations</td>
              <td>{{ result.iterations }}</td>
            </tr>
          </tbody>
        </v-table>
        <v-alert
          v-for="(warning, index) in result.warnings ?? []"
          :key="index"
          class="mb-2"
          color="warning"
          density="compact"
          icon="mdi-alert-outline"
          :text="warning"
          :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
        />

        <div v-for="comparison in cameraComparisons" :key="comparison.result.uniqueName" class="pt-3">
          <div class="d-flex align-center flex-wrap">
            <v-checkbox-btn
              v-model="camerasToSave"
              :value="comparison.result.uniqueName"
              :disabled="!result.success"
              color="primary"
              density="compact"
              class="flex-grow-0"
            />
            <span class="text-h6 pl-2">{{ comparison.result.nickname }}</span>
            <v-spacer />
            <span class="text-medium-emphasis">
              RMS
              {{
                Number.isFinite(comparison.result.rmsReprojectionErrorPx)
                  ? comparison.result.rmsReprojectionErrorPx.toFixed(2)
                  : "-"
              }}
              px &middot; {{ comparison.result.observations }} observations &middot; {{ comparison.result.stops }} stops
            </span>
          </div>
          <v-table density="compact" class="transform-table">
            <thead>
              <tr>
                <th />
                <th v-for="component in transformComponents" :key="component.key">{{ component.label }}</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td>Solved</td>
                <td v-for="component in transformComponents" :key="component.key">
                  {{ formatTransformComponent(comparison.result[component.key], component.isAngle) }}
                </td>
              </tr>
              <tr>
                <td>±1σ</td>
                <td v-for="(component, index) in transformComponents" :key="component.key">
                  {{ formatStdDev(comparison.result.stdDevs?.[index], component.isAngle) }}
                </td>
              </tr>
              <tr v-if="comparison.current">
                <td>
                  Current ({{ comparison.current.source }})
                  <div class="text-caption text-medium-emphasis">
                    {{ formatDate(comparison.current.updatedAtMillis) }}
                  </div>
                </td>
                <td v-for="component in transformComponents" :key="component.key">
                  {{ formatTransformComponent(comparison.current[component.key], component.isAngle) }}
                </td>
              </tr>
              <tr v-else>
                <td>Current</td>
                <td :colspan="transformComponents.length" class="text-medium-emphasis">
                  No robot-to-camera transform saved for this camera
                </td>
              </tr>
              <tr v-if="comparison.difference">
                <td>Difference</td>
                <td v-for="component in transformComponents" :key="component.key">
                  {{ formatDifference(comparison.difference[component.key], component.isAngle) }}
                </td>
              </tr>
            </tbody>
          </v-table>
          <v-alert
            v-for="(warning, index) in comparison.result.warnings ?? []"
            :key="index"
            class="mt-2"
            color="warning"
            density="compact"
            icon="mdi-alert-outline"
            :text="warning"
            :variant="theme.global.current.value.dark ? 'tonal' : 'elevated'"
          />
        </div>

        <v-expansion-panels v-if="(result.tags ?? []).length > 0" class="mt-4">
          <v-expansion-panel bg-color="surface">
            <v-expansion-panel-title>Tag Poses ({{ result.tags.length }})</v-expansion-panel-title>
            <v-expansion-panel-text>
              <v-table density="compact" class="transform-table">
                <thead>
                  <tr>
                    <th>ID</th>
                    <th>X</th>
                    <th>Y</th>
                    <th>Z</th>
                    <th>Roll</th>
                    <th>Pitch</th>
                    <th>Yaw</th>
                    <th>Observations</th>
                    <th>Notes</th>
                  </tr>
                </thead>
                <tbody>
                  <tr v-for="tag in result.tags" :key="tag.id">
                    <td>{{ tag.id }}</td>
                    <td>{{ formatMeters(tag.x) }}</td>
                    <td>{{ formatMeters(tag.y) }}</td>
                    <td>{{ formatMeters(tag.z) }}</td>
                    <td>{{ formatDegrees(tag.rollDeg) }}</td>
                    <td>{{ formatDegrees(tag.pitchDeg) }}</td>
                    <td>{{ formatDegrees(tag.yawDeg) }}</td>
                    <td>{{ tag.observations }}</td>
                    <td>{{ [tag.fixed ? "Fixed" : "", tag.floorTag ? "Floor" : ""].filter((n) => n).join(", ") }}</td>
                  </tr>
                </tbody>
              </v-table>
            </v-expansion-panel-text>
          </v-expansion-panel>
        </v-expansion-panels>

        <v-btn
          class="mt-4"
          block
          color="buttonActive"
          :variant="theme.global.current.value.dark ? 'outlined' : 'elevated'"
          :loading="robotCalibrationStore.committing"
          :disabled="!result.success || camerasToSave.length === 0 || !useStateStore().backendConnected"
          @click="saveToCameras"
        >
          <v-icon start size="large">mdi-content-save</v-icon>
          <span>Save to {{ camerasToSave.length }} camera{{ camerasToSave.length === 1 ? "" : "s" }}</span>
        </v-btn>
        <p v-if="!result.success" class="pt-2 text-medium-emphasis">
          The solve failed, so there is nothing to save. Address the problems above and solve again.
        </p>
        <p v-else class="pt-2 text-medium-emphasis">
          Saved transforms can be reviewed and edited on the
          <router-link to="/cameras">Camera</router-link> page.
        </p>
      </template>
    </v-card-text>
  </v-card>
</template>

<style scoped lang="scss">
.v-btn-toggle.fill {
  width: 100%;
  height: 100%;
}

.v-btn-toggle.fill > .v-btn {
  width: 50%;
  height: 100%;
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

.transform-table td {
  font-family: monospace;
  white-space: nowrap;
}

.transform-table td:first-child {
  font-family: inherit;
  text-align: left;
}

.summary-table td:first-child {
  text-align: left;
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
