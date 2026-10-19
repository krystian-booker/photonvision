import { defineStore } from "pinia";
import axios from "axios";
import { useStateStore } from "@/stores/StateStore";
import {
  DefaultExtrinsicCaptureSettings,
  DefaultExtrinsicSolverSettings,
  type ExtrinsicCalibrationStatus,
  type ExtrinsicCaptureSettings,
  type ExtrinsicSolveResult,
  type ExtrinsicSolverSettings
} from "@/types/RobotCalibrationTypes";

interface RobotCalibrationStore {
  // Form state for the capture and solve steps. The live session state lives in the StateStore.
  captureSettings: ExtrinsicCaptureSettings;
  solverSettings: ExtrinsicSolverSettings;
  // Until the user changes the camera selection, every camera is selected
  cameraSelectionTouched: boolean;
  solving: boolean;
  committing: boolean;
}

const isSolveResult = (data: unknown): data is ExtrinsicSolveResult =>
  typeof data === "object" && data !== null && "success" in data && "cameras" in data;

/**
 * Build a user facing message from a failed request. The backend replies to failed requests with a plain-text message.
 *
 * @param error the error thrown by axios.
 * @param description a brief description of the request, e.g., "start the capture".
 */
export const getRequestErrorMessage = (error: unknown, description: string): string => {
  if (axios.isAxiosError(error)) {
    if (error.response) {
      const data: unknown = error.response.data;
      if (typeof data === "string" && data.trim().length > 0) return data;
      if (typeof data === "object" && data !== null && "message" in data && typeof data.message === "string") {
        return data.message;
      }
      return `The backend is unable to ${description} (HTTP ${error.response.status}).`;
    }
    if (error.request) {
      return `Error while trying to ${description}! The backend didn't respond.`;
    }
  }
  return `An error occurred while trying to ${description}.`;
};

export const useRobotCalibrationStore = defineStore("robotCalibration", {
  state: (): RobotCalibrationStore => ({
    captureSettings: { ...DefaultExtrinsicCaptureSettings, cameras: [] },
    solverSettings: { ...DefaultExtrinsicSolverSettings, floorTagIds: [], knownTagHeights: [] },
    cameraSelectionTouched: false,
    solving: false,
    committing: false
  }),
  actions: {
    /**
     * Fetch the current session state. The websocket only pushes the state when it changes.
     */
    async fetchStatus() {
      try {
        const response = await axios.get<ExtrinsicCalibrationStatus>("/extrinsics/status");
        useStateStore().updateExtrinsicCalibrationStatus(response.data);
      } catch {
        // The backend may not be reachable yet, the websocket will update the state once it is
      }
    },
    /**
     * POST to an extrinsic calibration endpoint, reporting failures (and optionally successes) with the snackbar.
     *
     * @return whether the request succeeded
     */
    async postRequest(url: string, data: unknown, description: string, successMessage?: string): Promise<boolean> {
      try {
        await axios.post(url, data);
        if (successMessage) {
          useStateStore().showSnackbarMessage({ message: successMessage, color: "success" });
        }
        return true;
      } catch (error) {
        useStateStore().showSnackbarMessage({ message: getRequestErrorMessage(error, description), color: "error" });
        return false;
      } finally {
        await this.fetchStatus();
      }
    },
    startCapture() {
      return this.postRequest(
        "/extrinsics/start",
        this.captureSettings,
        "start the robot calibration capture",
        "Capture started. Drive the robot through the routine."
      );
    },
    pauseCapture() {
      return this.postRequest("/extrinsics/stop", {}, "pause the robot calibration capture", "Capture paused");
    },
    resetCapture() {
      return this.postRequest(
        "/extrinsics/reset",
        {},
        "reset the robot calibration session",
        "Robot calibration session reset"
      );
    },
    captureNow() {
      return this.postRequest("/extrinsics/captureNow", {}, "capture a stop");
    },
    removeStop(index: number) {
      return this.postRequest(
        "/extrinsics/removeStop",
        { index: index },
        "remove the stop",
        `Removed stop ${index + 1}`
      );
    },
    importDataset(data: unknown) {
      return this.postRequest(
        "/extrinsics/dataset",
        data,
        "import the robot calibration dataset",
        "Dataset imported. Press Solve to compute the camera transforms."
      );
    },
    /**
     * Solve for the robot-to-camera transforms of every camera in the session.
     *
     * @return the solve result, or undefined if the backend didn't produce one
     */
    async solve(): Promise<ExtrinsicSolveResult | undefined> {
      this.solving = true;
      let result: ExtrinsicSolveResult | undefined;
      try {
        const response = await axios.post<ExtrinsicSolveResult>("/extrinsics/solve", this.solverSettings);
        result = response.data;
        useStateStore().showSnackbarMessage({ message: result.message || "Solve succeeded", color: "success" });
      } catch (error) {
        // An unsuccessful solve still returns a result (HTTP 422) that explains what went wrong
        const data: unknown = axios.isAxiosError(error) ? error.response?.data : undefined;
        if (isSolveResult(data)) {
          result = data;
          useStateStore().showSnackbarMessage({ message: data.message || "Solve failed", color: "error" });
        } else {
          useStateStore().showSnackbarMessage({
            message: getRequestErrorMessage(error, "solve for the camera transforms"),
            color: "error"
          });
        }
      } finally {
        this.solving = false;
      }
      if (result) {
        // Show the result right away, the backend also broadcasts it with the session state
        const stateStore = useStateStore();
        stateStore.extrinsicCalibration = { ...stateStore.extrinsicCalibration, result: result };
      }
      return result;
    },
    /**
     * Save the solved robot-to-camera transforms to the camera configurations.
     *
     * @param cameras unique names of the cameras to save, or null to save every camera in the last result
     */
    async commit(cameras: string[] | null) {
      this.committing = true;
      try {
        await axios.post("/extrinsics/commit", { cameras: cameras });
        useStateStore().showSnackbarMessage({
          message: "Saved the robot-to-camera transforms to the cameras",
          color: "success"
        });
        return true;
      } catch (error) {
        useStateStore().showSnackbarMessage({
          message: getRequestErrorMessage(error, "save the robot-to-camera transforms"),
          color: "error"
        });
        return false;
      } finally {
        this.committing = false;
      }
    }
  }
});
