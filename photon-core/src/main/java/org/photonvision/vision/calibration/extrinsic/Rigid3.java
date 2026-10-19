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

package org.photonvision.vision.calibration.extrinsic;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A minimal, allocation-light rigid transform (rotation matrix + translation) used inside the
 * extrinsic solver's inner loops, where WPILib's immutable geometry classes would be too slow.
 *
 * <p>Rotation matrices are stored row-major in a double[9].
 */
final class Rigid3 {
    final double[] R;
    final double[] t;

    Rigid3(double[] R, double[] t) {
        this.R = R;
        this.t = t;
    }

    static Rigid3 identity() {
        return new Rigid3(new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1}, new double[3]);
    }

    Rigid3 copy() {
        return new Rigid3(R.clone(), t.clone());
    }

    /** Returns this * other. */
    Rigid3 compose(Rigid3 o) {
        double[] r = mul(R, o.R);
        double[] tt = new double[3];
        apply(R, o.t, tt);
        tt[0] += t[0];
        tt[1] += t[1];
        tt[2] += t[2];
        return new Rigid3(r, tt);
    }

    Rigid3 inverse() {
        double[] rt = transpose(R);
        double[] tt = new double[3];
        apply(rt, t, tt);
        tt[0] = -tt[0];
        tt[1] = -tt[1];
        tt[2] = -tt[2];
        return new Rigid3(rt, tt);
    }

    /** out = R * p + t. out may not alias p. */
    void transformPoint(double[] p, double[] out) {
        apply(R, p, out);
        out[0] += t[0];
        out[1] += t[1];
        out[2] += t[2];
    }

    /** Left-multiplies the rotation by Exp(w) in place. */
    void rotateLeft(double wx, double wy, double wz) {
        double[] e = expSO3(wx, wy, wz);
        System.arraycopy(mul(e, R), 0, R, 0, 9);
    }

    // ---------------- static helpers ----------------

    static void apply(double[] M, double[] v, double[] out) {
        out[0] = M[0] * v[0] + M[1] * v[1] + M[2] * v[2];
        out[1] = M[3] * v[0] + M[4] * v[1] + M[5] * v[2];
        out[2] = M[6] * v[0] + M[7] * v[1] + M[8] * v[2];
    }

    static double[] mul(double[] A, double[] B) {
        double[] C = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
            }
        }
        return C;
    }

    static double[] transpose(double[] A) {
        return new double[] {A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]};
    }

    static double[] rotZ(double theta) {
        double c = Math.cos(theta);
        double s = Math.sin(theta);
        return new double[] {c, -s, 0, s, c, 0, 0, 0, 1};
    }

    static double[] rotY(double theta) {
        double c = Math.cos(theta);
        double s = Math.sin(theta);
        return new double[] {c, 0, s, 0, 1, 0, -s, 0, c};
    }

    /** Rodrigues' formula. */
    static double[] expSO3(double wx, double wy, double wz) {
        double theta2 = wx * wx + wy * wy + wz * wz;
        double theta = Math.sqrt(theta2);
        double a;
        double b;
        if (theta < 1e-8) {
            a = 1 - theta2 / 6.0;
            b = 0.5 - theta2 / 24.0;
        } else {
            a = Math.sin(theta) / theta;
            b = (1 - Math.cos(theta)) / theta2;
        }
        // R = I + a*K + b*K^2, K = skew(w)
        return new double[] {
            1 - b * (wy * wy + wz * wz),
            -a * wz + b * wx * wy,
            a * wy + b * wx * wz,
            a * wz + b * wx * wy,
            1 - b * (wx * wx + wz * wz),
            -a * wx + b * wy * wz,
            -a * wy + b * wx * wz,
            a * wx + b * wy * wz,
            1 - b * (wx * wx + wy * wy)
        };
    }

    /** Inverse of {@link #expSO3}: returns the rotation vector of R. */
    static double[] logSO3(double[] R) {
        double cos = (R[0] + R[4] + R[8] - 1) / 2.0;
        cos = Math.max(-1, Math.min(1, cos));
        double theta = Math.acos(cos);
        double vx = R[7] - R[5];
        double vy = R[2] - R[6];
        double vz = R[3] - R[1];
        if (theta < 1e-6) {
            return new double[] {vx / 2, vy / 2, vz / 2};
        }
        if (Math.PI - theta < 1e-4) {
            // Near pi: recover the axis from the diagonal
            double xx = Math.sqrt(Math.max(0, (R[0] + 1) / 2));
            double yy = Math.sqrt(Math.max(0, (R[4] + 1) / 2));
            double zz = Math.sqrt(Math.max(0, (R[8] + 1) / 2));
            if (xx >= yy && xx >= zz) {
                yy = Math.copySign(yy, R[1] + R[3]);
                zz = Math.copySign(zz, R[2] + R[6]);
            } else if (yy >= zz) {
                xx = Math.copySign(xx, R[1] + R[3]);
                zz = Math.copySign(zz, R[5] + R[7]);
            } else {
                xx = Math.copySign(xx, R[2] + R[6]);
                yy = Math.copySign(yy, R[5] + R[7]);
            }
            double n = Math.sqrt(xx * xx + yy * yy + zz * zz);
            return new double[] {theta * xx / n, theta * yy / n, theta * zz / n};
        }
        double k = theta / (2 * Math.sin(theta));
        return new double[] {k * vx, k * vy, k * vz};
    }

    static double[] fromQuaternion(Quaternion q) {
        double w = q.getW();
        double x = q.getX();
        double y = q.getY();
        double z = q.getZ();
        double n = Math.sqrt(w * w + x * x + y * y + z * z);
        w /= n;
        x /= n;
        y /= n;
        z /= n;
        return new double[] {
            1 - 2 * (y * y + z * z),
            2 * (x * y - z * w),
            2 * (x * z + y * w),
            2 * (x * y + z * w),
            1 - 2 * (x * x + z * z),
            2 * (y * z - x * w),
            2 * (x * z - y * w),
            2 * (y * z + x * w),
            1 - 2 * (x * x + y * y)
        };
    }

    static Rotation3d toRotation3d(double[] R) {
        double[] w = logSO3(R);
        double angle = Math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
        if (angle < 1e-12) return new Rotation3d();
        double s = Math.sin(angle / 2) / angle;
        return new Rotation3d(new Quaternion(Math.cos(angle / 2), w[0] * s, w[1] * s, w[2] * s));
    }

    static Rigid3 fromPose(Pose3d pose) {
        return new Rigid3(
                fromQuaternion(pose.getRotation().getQuaternion()),
                new double[] {pose.getX(), pose.getY(), pose.getZ()});
    }

    static Rigid3 fromTransform(Transform3d tf) {
        return new Rigid3(
                fromQuaternion(tf.getRotation().getQuaternion()),
                new double[] {tf.getX(), tf.getY(), tf.getZ()});
    }

    Pose3d toPose() {
        return new Pose3d(new Translation3d(t[0], t[1], t[2]), toRotation3d(R));
    }

    Transform3d toTransform() {
        return new Transform3d(new Translation3d(t[0], t[1], t[2]), toRotation3d(R));
    }

    /** A planar robot pose (x, y, theta) lifted onto the floor plane. */
    static Rigid3 planar(double x, double y, double theta) {
        return new Rigid3(rotZ(theta), new double[] {x, y, 0});
    }

    /** Heading (rotation about +Z) of this transform's x axis projected onto the floor. */
    double yaw() {
        return Math.atan2(R[3], R[0]);
    }
}
