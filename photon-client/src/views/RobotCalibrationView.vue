<script setup lang="ts">
import { onMounted, watch } from "vue";
import { useStateStore } from "@/stores/StateStore";
import { useRobotCalibrationStore } from "@/stores/RobotCalibrationStore";
import RobotCalibrationSetupCard from "@/components/robot-calibration/RobotCalibrationSetupCard.vue";
import RobotCalibrationCaptureCard from "@/components/robot-calibration/RobotCalibrationCaptureCard.vue";
import RobotCalibrationSolveCard from "@/components/robot-calibration/RobotCalibrationSolveCard.vue";

// The websocket only pushes the session state when it changes, so fetch it when the page is opened
// and whenever the backend (re)connects
onMounted(() => {
  void useRobotCalibrationStore().fetchStatus();
});
watch(
  () => useStateStore().backendConnected,
  (connected) => {
    if (connected) void useRobotCalibrationStore().fetchStatus();
  }
);
</script>

<template>
  <div class="pa-3">
    <v-row no-gutters>
      <v-col cols="12" lg="6" class="pr-lg-3">
        <RobotCalibrationSetupCard />
        <RobotCalibrationCaptureCard />
      </v-col>
      <v-col cols="12" lg="6">
        <RobotCalibrationSolveCard />
      </v-col>
    </v-row>
  </div>
</template>
