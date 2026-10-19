import type { RobotToCameraTransform } from "@/types/SettingTypes";

export type TransformComponent = keyof RobotToCameraTransform;

// Display order of the transform components, matching the order of the backend's stdDevs arrays
export const transformComponents: { key: TransformComponent; label: string; isAngle: boolean }[] = [
  { key: "x", label: "X", isAngle: false },
  { key: "y", label: "Y", isAngle: false },
  { key: "z", label: "Z", isAngle: false },
  { key: "rollDeg", label: "Roll", isAngle: true },
  { key: "pitchDeg", label: "Pitch", isAngle: true },
  { key: "yawDeg", label: "Yaw", isAngle: true }
];

const isFiniteNumber = (value: number | undefined | null): value is number =>
  value !== undefined && value !== null && Number.isFinite(value);

export const formatMeters = (value: number | undefined | null): string =>
  isFiniteNumber(value) ? `${value.toFixed(3)} m` : "-";

export const formatDegrees = (value: number | undefined | null): string =>
  isFiniteNumber(value) ? `${value.toFixed(2)}°` : "-";

export const formatTransformComponent = (value: number | undefined | null, isAngle: boolean): string =>
  isAngle ? formatDegrees(value) : formatMeters(value);

/**
 * Format a 1-sigma uncertainty. Translations are given in meters and shown in millimeters, rotations are in degrees.
 */
export const formatStdDev = (value: number | undefined | null, isAngle: boolean): string => {
  if (!isFiniteNumber(value)) return "-";
  return isAngle ? `±${value.toFixed(2)}°` : `±${(value * 1000).toFixed(1)} mm`;
};

/**
 * Wrap an angle in degrees to [-180, 180).
 */
export const wrapDegrees = (value: number): number => {
  const wrapped = (((value + 180) % 360) + 360) % 360;
  return wrapped - 180;
};

/**
 * Component-wise difference a - b, wrapping the angle components.
 */
export const transformDifference = (
  a: RobotToCameraTransform,
  b: RobotToCameraTransform
): Record<TransformComponent, number> => ({
  x: a.x - b.x,
  y: a.y - b.y,
  z: a.z - b.z,
  rollDeg: wrapDegrees(a.rollDeg - b.rollDeg),
  pitchDeg: wrapDegrees(a.pitchDeg - b.pitchDeg),
  yawDeg: wrapDegrees(a.yawDeg - b.yawDeg)
});

/**
 * Format a difference, with translations in millimeters and rotations in degrees.
 */
export const formatDifference = (value: number, isAngle: boolean): string => {
  if (!Number.isFinite(value)) return "-";
  const sign = value > 0 ? "+" : "";
  return isAngle ? `${sign}${value.toFixed(2)}°` : `${sign}${(value * 1000).toFixed(1)} mm`;
};
