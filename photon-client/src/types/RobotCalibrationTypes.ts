// Companions to the robot extrinsic calibration classes in Java

export type ExtrinsicCapturePhase = "IDLE" | "WAITING_FOR_STILL" | "COLLECTING" | "WAITING_FOR_MOTION";

export interface ExtrinsicCaptureSettings {
  // Unique names of the cameras to capture from
  cameras: string[];
  tagSizeMeters: number;
  minTranslationMeters: number;
  minRotationDegrees: number;
  settleSeconds: number;
  framesPerStop: number;
}

export interface ExtrinsicCameraStatus {
  uniqueName: string;
  nickname: string;
  hasCalibration: boolean;
  // Formatted as "1280x800"
  resolution: string;
  tagsVisible: number;
  observations: number;
  stopsWithTags: number;
}

export interface ExtrinsicCameraResult {
  uniqueName: string;
  nickname: string;
  x: number;
  y: number;
  z: number;
  rollDeg: number;
  pitchDeg: number;
  yawDeg: number;
  // 1-sigma uncertainties: [x m, y m, z m, roll deg, pitch deg, yaw deg]
  stdDevs: number[];
  rmsReprojectionErrorPx: number;
  observations: number;
  stops: number;
  warnings: string[];
}

export interface ExtrinsicTagResult {
  id: number;
  x: number;
  y: number;
  z: number;
  rollDeg: number;
  pitchDeg: number;
  yawDeg: number;
  floorTag: boolean;
  fixed: boolean;
  observations: number;
}

export interface ExtrinsicSolveResult {
  success: boolean;
  message: string;
  cameras: ExtrinsicCameraResult[];
  tags: ExtrinsicTagResult[];
  odometryScale: number;
  rmsReprojectionErrorPx: number;
  iterations: number;
  stopsUsed: number;
  warnings: string[];
}

export interface ExtrinsicCalibrationStatus {
  phase: ExtrinsicCapturePhase;
  // Whether robot code is publishing odometry
  odometryConnected: boolean;
  robotStill: boolean;
  // [x m, y m, heading deg], or empty if no odometry has been received
  odometry: number[];
  stopCount: number;
  // Number of 30 degree heading buckets covered by stops with tags, out of 12
  headingsCovered: number;
  settings: ExtrinsicCaptureSettings;
  cameras: ExtrinsicCameraStatus[];
  // Latest human readable event
  message: string;
  // Omitted by the backend when no solve has been run
  result?: ExtrinsicSolveResult;
}

export interface KnownTagHeight {
  id: number;
  heightMeters: number;
}

export interface ExtrinsicSolverSettings {
  // True to use the field layout configured in Settings (tags fixed), false to self-survey the tags
  useFieldLayout: boolean;
  // Self-survey only: tags lying flat on the floor, face up
  floorTagIds: number[];
  floorTagHeightMeters: number;
  // Self-survey only: tags with a tape-measured center height
  knownTagHeights: KnownTagHeight[];
  pixelStdDev: number;
  odomTranslationStdDev: number;
  odomTranslationStdDevPerMeter: number;
  odomRotationStdDev: number;
  odomRotationStdDevPerRadian: number;
  estimateOdometryScale: boolean;
  huberThresholdPx: number;
  maxIterations: number;
}

export const HEADING_BUCKET_COUNT = 12;

export const DefaultExtrinsicCaptureSettings: ExtrinsicCaptureSettings = {
  cameras: [],
  // 6.5 inches
  tagSizeMeters: 0.1651,
  minTranslationMeters: 0.15,
  minRotationDegrees: 15,
  settleSeconds: 0.5,
  framesPerStop: 5
};

export const DefaultExtrinsicSolverSettings: ExtrinsicSolverSettings = {
  useFieldLayout: false,
  floorTagIds: [],
  floorTagHeightMeters: 0,
  knownTagHeights: [],
  pixelStdDev: 1.0,
  odomTranslationStdDev: 0.005,
  odomTranslationStdDevPerMeter: 0.02,
  odomRotationStdDev: 0.005236,
  odomRotationStdDevPerRadian: 0.01,
  estimateOdometryScale: true,
  huberThresholdPx: 3.0,
  maxIterations: 100
};

export const DefaultExtrinsicCalibrationStatus: ExtrinsicCalibrationStatus = {
  phase: "IDLE",
  odometryConnected: false,
  robotStill: false,
  odometry: [],
  stopCount: 0,
  headingsCovered: 0,
  settings: { ...DefaultExtrinsicCaptureSettings },
  cameras: [],
  message: ""
};
