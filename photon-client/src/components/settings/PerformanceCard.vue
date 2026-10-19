<script setup lang="ts">
import { computed } from "vue";
import PvSelect from "@/components/common/pv-select.vue";
import PvSwitch from "@/components/common/pv-switch.vue";
import PvSlider from "@/components/common/pv-slider.vue";
import { useSettingsStore } from "@/stores/settings/GeneralSettingsStore";
import { axiosPost } from "@/lib/PhotonUtils";
import type { AprilTagAcceleration } from "@/types/SettingTypes";

const accelerationItems: { name: string; value: AprilTagAcceleration }[] = [
  { name: "Auto (GPU on Rubik Pi 3, otherwise CPU)", value: "AUTO" },
  { name: "GPU (Vulkan), falling back to CPU", value: "GPU" },
  { name: "CPU only", value: "CPU" }
];

const savePerformance = (change: {
  aprilTagAcceleration?: AprilTagAcceleration;
  grayscaleCapture?: boolean;
  maxStreamFps?: number;
}) => axiosPost("/settings/performance", "save performance settings", change);

const acceleration = computed<AprilTagAcceleration>({
  get: () => useSettingsStore().general.aprilTagAcceleration,
  set: (v) => {
    useSettingsStore().general.aprilTagAcceleration = v;
    void savePerformance({ aprilTagAcceleration: v });
  }
});

const grayscaleCapture = computed<boolean>({
  get: () => useSettingsStore().general.grayscaleCapture,
  set: (v) => {
    useSettingsStore().general.grayscaleCapture = v;
    void savePerformance({ grayscaleCapture: v });
  }
});

const maxStreamFps = computed<number>({
  get: () => useSettingsStore().general.maxStreamFps,
  set: (v) => {
    useSettingsStore().general.maxStreamFps = v;
  }
});
</script>

<template>
  <v-card class="mb-3 rounded-12" color="surface">
    <v-card-title class="pb-10px">Performance</v-card-title>
    <v-card-text class="pt-0">
      <pv-select
        v-model="acceleration"
        label="AprilTag Detection"
        tooltip="Where AprilTag detection runs. The GPU path does the image processing on the GPU (Vulkan) and only tag decoding on the CPU, leaving CPU time for more cameras or higher frame rates. If the GPU can't be used, detection falls back to the CPU automatically."
        :items="accelerationItems"
        :select-cols="8"
      />
      <div class="d-flex">
        <v-col cols="4" class="pl-0 py-1 d-flex align-center text-body-2">GPU Status</v-col>
        <v-col cols="8" class="pr-0 py-1 d-flex align-center text-body-2">
          <v-icon
            class="mr-2"
            size="small"
            :color="useSettingsStore().general.gpuAprilTagAvailable ? 'success' : 'grey'"
          >
            {{ useSettingsStore().general.gpuAprilTagAvailable ? "mdi-check-circle" : "mdi-minus-circle" }}
          </v-icon>
          <span>{{ useSettingsStore().general.gpuAprilTagStatus || "Unknown" }}</span>
        </v-col>
      </div>
      <pv-switch
        v-model="grayscaleCapture"
        label="Grayscale Capture"
        tooltip="For AprilTag and ArUco pipelines, decode camera frames straight to grayscale instead of color. Saves CPU, especially with monochrome cameras like the OV9281. The input stream is shown in grayscale for these pipelines."
        :label-cols="4"
        :switch-cols="8"
      />
      <pv-slider
        v-model="maxStreamFps"
        label="Max Stream FPS"
        tooltip="How often the dashboard camera streams are drawn and encoded. Vision results sent to the robot are not limited. 0 = unlimited."
        :min="0"
        :max="120"
        :slider-cols="8"
        @update:modelValue="(v) => savePerformance({ maxStreamFps: v })"
      />
    </v-card-text>
  </v-card>
</template>
