// SO-101アームの数値逆運動学(IK)のC++実装。
//
// so101_ik.py の solve_ik() / end_effector_pose() と同じアルゴリズム
// (数値ヤコビアン + 疑似逆行列によるGauss-Newton法)をC++で実装し、
// pybind11でPythonから呼び出せるようにしたもの。
//
// pinv(J)はJacobi法による4x4 SVDを用いてrcond付きで計算し、
// numpyのnp.linalg.pinv(J, rcond=...)と同等の打ち切りを行う。
//
// solve_ik()の計算中はGILを解放するため、バックグラウンドスレッドで
// 実行してもGUIスレッド(他のPythonコード)をブロックしない。

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <cmath>

namespace py = pybind11;

namespace {

constexpr double PI = 3.14159265358979323846;

inline double deg2rad(double deg) { return deg * PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / PI; }

// 4x4同次変換行列。
struct Mat4 {
    std::array<std::array<double, 4>, 4> m{};

    static Mat4 identity() {
        Mat4 r;
        for (int i = 0; i < 4; ++i) r.m[i][i] = 1.0;
        return r;
    }

    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                double s = 0.0;
                for (int k = 0; k < 4; ++k) s += m[i][k] * o.m[k][j];
                r.m[i][j] = s;
            }
        return r;
    }
};

Mat4 translate(double x, double y, double z) {
    Mat4 r = Mat4::identity();
    r.m[0][3] = x;
    r.m[1][3] = y;
    r.m[2][3] = z;
    return r;
}

Mat4 rot_x(double t) {
    Mat4 r = Mat4::identity();
    double c = std::cos(t), s = std::sin(t);
    r.m[1][1] = c; r.m[1][2] = -s;
    r.m[2][1] = s; r.m[2][2] = c;
    return r;
}

Mat4 rot_y(double t) {
    Mat4 r = Mat4::identity();
    double c = std::cos(t), s = std::sin(t);
    r.m[0][0] = c;  r.m[0][2] = s;
    r.m[2][0] = -s; r.m[2][2] = c;
    return r;
}

Mat4 rot_z(double t) {
    Mat4 r = Mat4::identity();
    double c = std::cos(t), s = std::sin(t);
    r.m[0][0] = c; r.m[0][1] = -s;
    r.m[1][0] = s; r.m[1][1] = c;
    return r;
}

// URDFのorigin rpyと同じ規約: R = Rz(yaw) * Ry(pitch) * Rx(roll)
Mat4 rpy_matrix(double roll, double pitch, double yaw) {
    return rot_z(yaw) * rot_y(pitch) * rot_x(roll);
}

// --- so101_kinematics.py の JOINT_PARAMS (shoulder_pan 〜 wrist_roll) ---
struct JointParam {
    double xyz[3];
    double rpy[3];
    double limit_lo_rad;
    double limit_hi_rad;
};

const JointParam kJointParams[5] = {
    // shoulder_pan
    {{0.0388353, -8.97657e-09, 0.0624}, {3.14159, 4.18253e-17, -3.14159}, -1.91986, 1.91986},
    // shoulder_lift
    {{-0.0303992, -0.0182778, -0.0542}, {-1.5708, -1.5708, 0.0}, -1.74533, 1.74533},
    // elbow_flex
    {{-0.11257, -0.028, 1.73763e-16}, {-3.63608e-16, 8.74301e-16, 1.5708}, -1.69, 1.69},
    // wrist_flex
    {{-0.1349, 0.0052, 3.62355e-17}, {4.02456e-15, 8.67362e-16, -1.5708}, -1.65806, 1.65806},
    // wrist_roll
    {{5.55112e-17, -0.0611, 0.0181}, {1.5708, 0.0486795, 3.14159}, -2.74385, 2.84121},
};

// gripper_link -> TCP (gripper_frame_link) の固定変換 (fixed joint)
const double kGripperFrameXyz[3] = {-0.0079, -0.000218121, -0.0981274};
const double kGripperFrameRpy[3] = {0.0, 3.14159, 0.0};

// 関節originの固定変換 (joint_transform = static_part * rot_z(theta))
struct StaticParts {
    std::array<Mat4, 5> joint;
    Mat4 gripper_frame;

    StaticParts() {
        for (int i = 0; i < 5; ++i) {
            const auto& p = kJointParams[i];
            joint[i] = translate(p.xyz[0], p.xyz[1], p.xyz[2]) *
                       rpy_matrix(p.rpy[0], p.rpy[1], p.rpy[2]);
        }
        gripper_frame = translate(kGripperFrameXyz[0], kGripperFrameXyz[1], kGripperFrameXyz[2]) *
                        rpy_matrix(kGripperFrameRpy[0], kGripperFrameRpy[1], kGripperFrameRpy[2]);
    }
};

const StaticParts kStatic;

// 4要素ベクトル (x, y, z, pitch[rad])
using Vec4 = std::array<double, 4>;

// IKで解く4関節 (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex) +
// wrist_roll の角度[deg]からTCP位置とピッチ角を計算する。
// (gripperの開閉角はTCP位置・姿勢に影響しないため引数に取らない)
Vec4 end_effector_pose(const std::array<double, 4>& q4_deg, double wrist_roll_deg) {
    Mat4 T = Mat4::identity();
    for (int i = 0; i < 4; ++i) {
        T = T * kStatic.joint[i] * rot_z(deg2rad(q4_deg[i]));
    }
    T = T * kStatic.joint[4] * rot_z(deg2rad(wrist_roll_deg));

    const Mat4& T_gripper_link = T;
    Mat4 end_effector_T = T_gripper_link * kStatic.gripper_frame;

    double pos[3] = {end_effector_T.m[0][3], end_effector_T.m[1][3], end_effector_T.m[2][3]};

    // wrist_roll回転軸 (T_gripper_linkのZ軸)。TCP方向を向くように符号を揃える。
    double finger_axis[3] = {
        T_gripper_link.m[0][2], T_gripper_link.m[1][2], T_gripper_link.m[2][2],
    };
    double diff[3] = {
        pos[0] - T_gripper_link.m[0][3],
        pos[1] - T_gripper_link.m[1][3],
        pos[2] - T_gripper_link.m[2][3],
    };
    double dot = finger_axis[0] * diff[0] + finger_axis[1] * diff[1] + finger_axis[2] * diff[2];
    if (dot < 0.0) {
        finger_axis[0] = -finger_axis[0];
        finger_axis[1] = -finger_axis[1];
        finger_axis[2] = -finger_axis[2];
    }

    double horizontal = std::hypot(finger_axis[0], finger_axis[1]);
    double pitch = std::atan2(finger_axis[2], horizontal);

    return {pos[0], pos[1], pos[2], pitch};
}

// --- 4x4疑似逆行列 (one-sided Jacobi SVD, rcondによる特異値の打ち切り) ---

struct Mat4x4Plain {
    double m[4][4];
};

Mat4x4Plain mat_identity4() {
    Mat4x4Plain r{};
    for (int i = 0; i < 4; ++i) r.m[i][i] = 1.0;
    return r;
}

// A (4x4) の疑似逆行列をone-sided Jacobi SVDで計算する。
// A = U * diag(sigma) * V^T として、pinv(A) = V * diag(sigma^+) * U^T。
// sigma_j <= rcond * max(sigma) の特異値は0として扱う(numpyのpinvと同様)。
Mat4x4Plain pinv4x4(const Mat4x4Plain& A_in, double rcond) {
    Mat4x4Plain A = A_in;
    Mat4x4Plain V = mat_identity4();

    constexpr int kMaxSweeps = 60;
    constexpr double kEps = 1e-14;

    for (int sweep = 0; sweep < kMaxSweeps; ++sweep) {
        double off_diag_sq = 0.0;
        for (int p = 0; p < 3; ++p) {
            for (int q = p + 1; q < 4; ++q) {
                double alpha = 0.0, beta = 0.0, gamma = 0.0;
                for (int i = 0; i < 4; ++i) {
                    alpha += A.m[i][p] * A.m[i][p];
                    beta += A.m[i][q] * A.m[i][q];
                    gamma += A.m[i][p] * A.m[i][q];
                }
                off_diag_sq += gamma * gamma;
                if (std::abs(gamma) <= kEps * std::sqrt(alpha * beta + kEps)) continue;

                double zeta = (beta - alpha) / (2.0 * gamma);
                double t = (zeta >= 0.0 ? 1.0 : -1.0) /
                           (std::abs(zeta) + std::sqrt(1.0 + zeta * zeta));
                double c = 1.0 / std::sqrt(1.0 + t * t);
                double s = c * t;

                for (int i = 0; i < 4; ++i) {
                    double aip = A.m[i][p], aiq = A.m[i][q];
                    A.m[i][p] = c * aip - s * aiq;
                    A.m[i][q] = s * aip + c * aiq;

                    double vip = V.m[i][p], viq = V.m[i][q];
                    V.m[i][p] = c * vip - s * viq;
                    V.m[i][q] = s * vip + c * viq;
                }
            }
        }
        if (off_diag_sq < kEps * kEps) break;
    }

    // 特異値 = Aの各列のノルム。U = A / sigma (列ごと)。
    double sigma[4];
    double sigma_max = 0.0;
    for (int j = 0; j < 4; ++j) {
        double s = 0.0;
        for (int i = 0; i < 4; ++i) s += A.m[i][j] * A.m[i][j];
        sigma[j] = std::sqrt(s);
        sigma_max = std::max(sigma_max, sigma[j]);
    }

    double sigma_inv[4];
    double threshold = rcond * sigma_max;
    for (int j = 0; j < 4; ++j) {
        sigma_inv[j] = (sigma[j] > threshold && sigma[j] > 0.0) ? (1.0 / sigma[j]) : 0.0;
    }

    // U^T の各行 j = A の列 j / sigma[j] (sigma[j]==0なら0ベクトル)
    // pinv(A) = V * diag(sigma_inv) * U^T
    //         = sum_j sigma_inv[j] * (V列j) * (U列j)^T
    //         = sum_j (sigma_inv[j] / sigma[j]) * (V列j) * (A列j)^T   (sigma[j]>0のとき)
    Mat4x4Plain result{};
    for (int j = 0; j < 4; ++j) {
        if (sigma_inv[j] == 0.0) continue;
        double scale = sigma_inv[j] / sigma[j];
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                result.m[r][c] += scale * V.m[r][j] * A.m[c][j];
            }
        }
    }
    return result;
}

// --- so101_ik.py の各種定数 ---
constexpr double kFdEpsDeg = 0.5;       // 数値微分のステップ幅[deg]
constexpr double kPinvRcond = 1e-3;     // 疑似逆行列の特異値カットオフ
constexpr double kMaxStepDeg = 15.0;    // 1反復あたりの角度更新量の上限[deg]

std::array<double, 4> clamp_q4(const std::array<double, 4>& q) {
    std::array<double, 4> r = q;
    for (int i = 0; i < 4; ++i) {
        double lo = rad2deg(kJointParams[i].limit_lo_rad);
        double hi = rad2deg(kJointParams[i].limit_hi_rad);
        r[i] = std::min(std::max(r[i], lo), hi);
    }
    return r;
}

// 数値ヤコビアン (J[:,i] = (g(q + eps*e_i) - g(q)) / eps_deg) とg(q)を計算する。
void numerical_jacobian(const std::array<double, 4>& q4_deg, double wrist_roll_deg,
                         Mat4x4Plain& J, Vec4& g0) {
    g0 = end_effector_pose(q4_deg, wrist_roll_deg);
    for (int i = 0; i < 4; ++i) {
        std::array<double, 4> q_perturbed = q4_deg;
        q_perturbed[i] += kFdEpsDeg;
        Vec4 g1 = end_effector_pose(q_perturbed, wrist_roll_deg);
        for (int k = 0; k < 4; ++k) {
            J.m[k][i] = (g1[k] - g0[k]) / kFdEpsDeg;
        }
    }
}

double error_norm(const Vec4& err) {
    double s = 0.0;
    for (double v : err) s += v * v;
    return std::sqrt(s);
}

// solve_ik() 本体。so101_ik.py の solve_ik() と同じアルゴリズム。
std::array<double, 4> solve_ik_impl(const Vec4& target, std::array<double, 4> q4_init_deg,
                                     double wrist_roll_deg, int max_iters, double tol_pos,
                                     double tol_pitch) {
    std::array<double, 4> q = clamp_q4(q4_init_deg);
    std::array<double, 4> best_q = q;
    double best_err_norm = -1.0;

    for (int iter = 0; iter < max_iters; ++iter) {
        Mat4x4Plain J{};
        Vec4 g{};
        numerical_jacobian(q, wrist_roll_deg, J, g);

        Vec4 err{};
        for (int k = 0; k < 4; ++k) err[k] = target[k] - g[k];
        double err_n = error_norm(err);

        if (best_err_norm < 0.0 || err_n < best_err_norm) {
            best_q = q;
            best_err_norm = err_n;
        }

        double pos_err_n = std::sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);
        if (pos_err_n < tol_pos && std::abs(err[3]) < tol_pitch) break;

        Mat4x4Plain Jpinv = pinv4x4(J, kPinvRcond);

        // delta_rad = pinv(J) @ err  (so101_ik.pyの命名を踏襲。実体は[deg]相当)
        double delta_rad[4] = {0, 0, 0, 0};
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) delta_rad[r] += Jpinv.m[r][c] * err[c];

        double delta_deg[4];
        double step = 0.0;
        for (int i = 0; i < 4; ++i) {
            delta_deg[i] = rad2deg(delta_rad[i]);
            step = std::max(step, std::abs(delta_deg[i]));
        }
        if (step > kMaxStepDeg) {
            double scale = kMaxStepDeg / step;
            for (int i = 0; i < 4; ++i) delta_deg[i] *= scale;
        }

        std::array<double, 4> q_next;
        for (int i = 0; i < 4; ++i) q_next[i] = q[i] + delta_deg[i];
        q = clamp_q4(q_next);
    }

    return best_q;
}

}  // namespace

PYBIND11_MODULE(so101_ik_cpp, m) {
    m.doc() = "SO-101 IK (numerical Jacobian Gauss-Newton) implemented in C++";

    m.def(
        "end_effector_pose",
        [](const std::array<double, 4>& q4_deg, double wrist_roll_deg, double /*gripper_deg*/) {
            return end_effector_pose(q4_deg, wrist_roll_deg);
        },
        py::arg("q4_deg"), py::arg("wrist_roll_deg"), py::arg("gripper_deg") = 0.0,
        "4関節角度[deg]とwrist_roll[deg]からTCP位置[x,y,z]とピッチ角[rad]を返す。");

    m.def(
        "solve_ik",
        [](const Vec4& target, const std::array<double, 4>& q4_init_deg, double wrist_roll_deg,
           double /*gripper_deg*/, int max_iters, double tol_pos, double tol_pitch) {
            py::gil_scoped_release release;  // 計算中はGILを解放する
            return solve_ik_impl(target, q4_init_deg, wrist_roll_deg, max_iters, tol_pos,
                                  tol_pitch);
        },
        py::arg("target"), py::arg("q4_init_deg"), py::arg("wrist_roll_deg"),
        py::arg("gripper_deg") = 0.0, py::arg("max_iters") = 50, py::arg("tol_pos") = 1e-4,
        py::arg("tol_pitch") = 1e-3,
        "目標[x,y,z,pitch(rad)]に最も近い4関節角度[deg]を反復計算で求める。");
}
