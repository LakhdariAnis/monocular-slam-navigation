/**
 * slam_reader.cc — Dual-mode ORB-SLAM3 monocular tracker
 *
 *   --map             Mapping mode: build a fresh map from camera feed.
 *                     On Ctrl+C saves KeyFrameTrajectory.txt, MapPoints.txt,
 *                     MarkerPositions.txt (SE3 format), atlas (room_map.osa),
 *                     then auto-runs map_builder.py.
 *
 *   --loc <atlas>     Localization mode: load a saved Atlas (.osa),
 *                     relocalize and stream live pose over ZMQ.
 *                     No files saved on exit.
 *
 * Build flags:
 *   -DWITH_ARUCO        Enable ArUco marker detection and SE3 relocalization
 *   -DWITH_ZMQ          Enable ZMQ pose publisher
 *   -DWITH_POSE_INJECT  Enable InjectPoseHint (requires ORB-SLAM3 patch)
 *
 * Usage:
 *   ./slam_reader <ORBvoc.txt> <picam.yaml> --map     [--no-viewer]
 *   ./slam_reader <ORBvoc.txt> <picam.yaml> --loc room_map  [--no-viewer]
 *
 * IMPORTANT: After applying this update you MUST remap.
 *   MarkerPositions.txt now stores full SE3 (position + quaternion).
 *   The old XYZ-only format cannot support accurate pose injection.
 *   The loader falls back gracefully with a warning if given the old format,
 *   but relocalization accuracy will be poor without rotation data.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#ifdef WITH_ARUCO
#include <opencv2/aruco.hpp>
#endif

#ifdef WITH_ZMQ
#include <zmq.h>
#endif

#include "System.h"
#include <sophus/se3.hpp>

// ── Shared memory constants (must match bridge.py) ────────────────────────
static constexpr int WIDTH        = 640;
static constexpr int HEIGHT       = 480;
static constexpr int CHANNELS     = 3;
static constexpr size_t FRAME_BYTES  = WIDTH * HEIGHT * CHANNELS;
static constexpr size_t HEADER_BYTES = 8 + 8;      // seq (uint64) + ts (double)
static constexpr size_t SHM_SIZE     = HEADER_BYTES + FRAME_BYTES;
static const char *SHM_NAME = "/orbframe";

// ── ArUco config ──────────────────────────────────────────────────────────
#ifdef WITH_ARUCO
static constexpr int   ARUCO_DICT_ID     = cv::aruco::DICT_4X4_50;
static constexpr int   ARUCO_CORNER_RADIUS = 6;          // px — must be same in mapping AND loc
static const cv::Scalar ARUCO_CORNER_COLOR(255, 255, 255);
static constexpr float MARKER_SIZE_M     = 0.184f;       // 18.4 cm physical size

// Full SE3 observation of one marker detection during mapping.
// Both position and orientation in world frame are stored so we can
// recover exact camera pose during localization via T_wc = T_wm * T_cm^{-1}.
struct MarkerObsSE3 {
    Eigen::Vector3f    pos_world;   // marker origin in world frame
    Eigen::Quaternionf rot_world;   // marker orientation in world frame
};
#endif  // WITH_ARUCO

// ── Run modes ─────────────────────────────────────────────────────────────
enum RunMode { MODE_NONE, MODE_MAP, MODE_LOC };

// ── Signal handler ────────────────────────────────────────────────────────
std::atomic<bool> g_stop{false};
void sig_handler(int) {
    std::cout << "\n[SIGNAL] Ctrl+C caught — initiating shutdown ...\n";
    g_stop.store(true);
}

// ── Low-level helpers ─────────────────────────────────────────────────────
static inline uint64_t read_u64(const uint8_t *p) {
    uint64_t v = 0;
    std::memcpy(&v, p, 8);
    return v;
}
static inline double read_f64(const uint8_t *p) {
    double v = 0.0;
    std::memcpy(&v, p, 8);
    return v;
}
static Eigen::Vector3f cam_position(const Sophus::SE3f &Tcw) {
    return Tcw.inverse().translation();
}

// ── JSON pose serialiser ──────────────────────────────────────────────────
static std::string pose_to_json(const Sophus::SE3f &Tcw, double ts,
                                uint64_t seq, bool ok) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "{\"seq\":" << seq << ",\"ts\":" << ts
       << ",\"ok\":" << (ok ? "true" : "false");
    if (ok) {
        Eigen::Vector3f t = cam_position(Tcw);
        Eigen::Matrix3f R = Tcw.inverse().rotationMatrix();
        ss << ",\"x\":" << t.x() << ",\"y\":" << t.y() << ",\"z\":" << t.z();
        ss << ",\"R\":[";
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                ss << R(r, c);
                if (r * 3 + c < 8) ss << ",";
            }
        ss << "]";
    } else {
        ss << ",\"x\":null,\"y\":null,\"z\":null,\"R\":null";
    }
    ss << "}";
    return ss.str();
}

// ── Save map points ──────────────────────────────────────────────────────
static void save_map_points(ORB_SLAM3::System &SLAM, const std::string &path) {
    auto *map = SLAM.GetAtlas()->GetCurrentMap();
    if (!map) {
        std::cerr << "[slam_reader] No active map — skipping MapPoints.\n";
        return;
    }
    const auto &points = map->GetAllMapPoints();
    std::ofstream f(path);
    if (!f.is_open()) { std::cerr << "[slam_reader] Cannot write " << path << "\n"; return; }

    int written = 0, skipped = 0;
    for (auto *mp : points) {
        if (!mp || mp->isBad()) { ++skipped; continue; }
        Eigen::Vector3f pos = mp->GetWorldPos();
        f << std::fixed << std::setprecision(6)
          << pos.x() << " " << pos.y() << " " << pos.z() << "\n";
        ++written;
    }
    std::cout << "[slam_reader] Saved " << written << " map points ("
              << skipped << " bad/null skipped) -> " << path << "\n";
}

// ── ArUco functions ──────────────────────────────────────────────────────
#ifdef WITH_ARUCO

// Save full SE3 per marker.
// Format: marker_id  tx ty tz  qw qx qy qz  num_observations
// Rotation is averaged via naive quaternion mean (safe for small angular
// variance, which is expected for a physically fixed marker).
static void save_marker_positions(
        const std::map<int, std::vector<MarkerObsSE3>> &observations,
        const std::string &path) {
    std::ofstream mf(path);
    if (!mf.is_open()) {
        std::cerr << "[slam_reader] Cannot open " << path << "\n";
        return;
    }
    mf << "# marker_id tx ty tz qw qx qy qz num_observations\n";
    int count = 0;
    for (const auto &pair : observations) {
        if (pair.second.empty()) continue;

        Eigen::Vector3f sum_pos = Eigen::Vector3f::Zero();
        Eigen::Vector4f sum_q   = Eigen::Vector4f::Zero();
        const Eigen::Quaternionf &ref_q = pair.second[0].rot_world;

        for (const auto &obs : pair.second) {
            sum_pos += obs.pos_world;
            Eigen::Quaternionf q = obs.rot_world;
            // Flip to same hemisphere before summing to avoid cancellation
            if (ref_q.coeffs().dot(q.coeffs()) < 0.0f)
                q.coeffs() = -q.coeffs();
            sum_q += q.coeffs();
        }

        Eigen::Vector3f    avg_pos = sum_pos / (float)pair.second.size();
        Eigen::Quaternionf avg_q;
        avg_q.coeffs() = sum_q.normalized();

        mf << pair.first << " " << std::fixed << std::setprecision(6)
           << avg_pos.x() << " " << avg_pos.y() << " " << avg_pos.z() << " "
           << avg_q.w()   << " " << avg_q.x()   << " " << avg_q.y()   << " " << avg_q.z()
           << " " << pair.second.size() << "\n";
        ++count;
        std::cout << "[slam_reader]   M" << pair.first
                  << "  pos=(" << avg_pos.transpose() << ")"
                  << "  obs=" << pair.second.size() << "\n";
    }
    std::cout << "[slam_reader] Saved " << count << " marker SE3s -> " << path << "\n";
}

// Load full SE3 per marker for pose injection during localization.
// Supports both:
//   New format:  id  tx ty tz  qw qx qy qz  obs
//   Old format:  id  x y z  obs                (warns, uses identity rotation)
static std::map<int, Sophus::SE3f>
load_marker_world_poses(const std::string &path) {
    std::map<int, Sophus::SE3f> result;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[slam_reader] Cannot open " << path
                  << " — ArUco pose injection disabled.\n";
        return result;
    }
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream ss(line);
        int id;
        float tx, ty, tz;
        if (!(ss >> id >> tx >> ty >> tz)) continue;

        float qw, qx, qy, qz;
        int obs;
        if (ss >> qw >> qx >> qy >> qz >> obs) {
            // New SE3 format
            Eigen::Quaternionf q(qw, qx, qy, qz);
            q.normalize();
            result[id] = Sophus::SE3f(q, Eigen::Vector3f(tx, ty, tz));
            std::cout << "[slam_reader] Loaded M" << id
                      << "  pos=(" << tx << "," << ty << "," << tz << ")  [SE3]\n";
        } else {
            // Old position-only format — rotation is unknown
            result[id] = Sophus::SE3f(Eigen::Quaternionf::Identity(),
                                      Eigen::Vector3f(tx, ty, tz));
            std::cerr << "[slam_reader] WARNING: M" << id
                      << " loaded without rotation (old format)."
                      << " Pose injection will be inaccurate — REMAP to fix.\n";
        }
    }
    std::cout << "[slam_reader] Loaded " << result.size()
              << " marker world poses.\n";
    return result;
}

// Stamp white squares over ArUco corners so they become strong ORB features.
// CRITICAL: always call with ARUCO_CORNER_RADIUS (same value used during
// mapping). Using a different radius produces different ORB descriptors
// that won't match the map keyframes in BoW search.
static void stamp_aruco_corners(
        cv::Mat &frame_bgr,
        const std::vector<std::vector<cv::Point2f>> &corners,
        int radius = ARUCO_CORNER_RADIUS) {
    for (const auto &mc : corners) {
        for (const auto &pt : mc) {
            int x = static_cast<int>(pt.x);
            int y = static_cast<int>(pt.y);
            cv::rectangle(frame_bgr,
                          cv::Point(x - radius, y - radius),
                          cv::Point(x + radius, y + radius),
                          ARUCO_CORNER_COLOR, cv::FILLED);
        }
    }
}

// Recover full 3×3 rotation from an OpenCV Rodrigues rvec.
static Eigen::Matrix3f rvec_to_eigen(const cv::Vec3d &rvec) {
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    Eigen::Matrix3f R;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R(r, c) = (float)R_cv.at<double>(r, c);
    return R;
}

#endif  // WITH_ARUCO

// ── Runtime YAML generator ────────────────────────────────────────────────
static std::string make_runtime_config(const std::string &base_config,
                                       RunMode mode,
                                       const std::string &atlas_name,
                                       const std::string &out_dir = ".") {
    const std::string runtime_path = out_dir + "/picam_runtime.yaml";
    std::ifstream in(base_config);
    if (!in.is_open()) {
        std::cerr << "[ERROR] Cannot open config: " << base_config << "\n";
        return "";
    }
    std::ofstream out(runtime_path);
    std::string line;
    while (std::getline(in, line)) {
        if (line.find("System.LoadAtlasFromFile") != std::string::npos ||
            line.find("System.SaveAtlasToFile")   != std::string::npos)
            continue;
        out << line << "\n";
    }
    out << "\n# ── Auto-generated by slam_reader ──\n";
    char abs_path[4096];
    if (mode == MODE_MAP) {
        if (realpath(out_dir.c_str(), abs_path))
            out << "System.SaveAtlasToFile: \"" << abs_path << "/" << atlas_name << "\"\n";
        else
            out << "System.SaveAtlasToFile: \"" << out_dir << "/" << atlas_name << "\"\n";
    } else if (mode == MODE_LOC) {
        if (atlas_name.find('/') == std::string::npos) {
            if (realpath(out_dir.c_str(), abs_path))
                out << "System.LoadAtlasFromFile: \"" << abs_path << "/" << atlas_name << "\"\n";
            else
                out << "System.LoadAtlasFromFile: \"" << out_dir << "/" << atlas_name << "\"\n";
        } else {
            out << "System.LoadAtlasFromFile: \"" << atlas_name << "\"\n";
        }
    }
    out.close();
    return runtime_path;
}

// ── Usage ─────────────────────────────────────────────────────────────────
static void print_usage(const char *prog) {
    std::cerr
        << "\nUsage:\n"
        << "  " << prog << " <ORBvoc.txt> <picam.yaml> --map  [--no-viewer]\n"
        << "  " << prog << " <ORBvoc.txt> <picam.yaml> --loc <atlas_name> [--no-viewer]\n\n"
        << "Build flags: -DWITH_ARUCO -DWITH_ZMQ -DWITH_POSE_INJECT\n\n";
}

// ══════════════════════════════════════════════════════════════════════════
//  Main
// ══════════════════════════════════════════════════════════════════════════
int main(int argc, char **argv) {
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    if (argc < 4) { print_usage(argv[0]); return 1; }

    const std::string vocab_path  = argv[1];
    const std::string config_path = argv[2];

    RunMode     mode       = MODE_NONE;
    std::string atlas_name = "room_map";
    std::string output_dir = ".";
    bool        use_viewer = true;

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--map") {
            mode = MODE_MAP;
        } else if (arg == "--loc") {
            mode = MODE_LOC;
            if (i + 1 < argc) atlas_name = argv[++i];
            else { std::cerr << "[ERROR] --loc requires an atlas name.\n"; return 1; }
        } else if (arg == "--output-dir") {
            if (i + 1 < argc) output_dir = argv[++i];
            else { std::cerr << "[ERROR] --output-dir requires a path.\n"; return 1; }
        } else if (arg == "--no-viewer") {
            use_viewer = false;
        } else {
            std::cerr << "[ERROR] Unknown argument: " << arg << "\n";
            print_usage(argv[0]); return 1;
        }
    }
    if (mode == MODE_NONE) {
        std::cerr << "[ERROR] Must specify --map or --loc <atlas>.\n";
        print_usage(argv[0]); return 1;
    }

    std::cout << "\n==================================================\n";
    if (mode == MODE_MAP)
        std::cout << "  MODE: MAPPING\n  Atlas: " << atlas_name << ".osa\n"
                  << "  Drive around, then Ctrl+C to save.\n";
    else
        std::cout << "  MODE: LOCALIZATION\n  Atlas: " << atlas_name << ".osa\n"
                  << "  Live pose streaming on ZMQ port 5557.\n";
    std::cout << "==================================================\n\n";

    // ── Runtime YAML ────────────────────────────────────────────────────
    std::string runtime_config =
        make_runtime_config(config_path, mode, atlas_name, output_dir);
    if (runtime_config.empty()) return 1;

    // ── ArUco setup ──────────────────────────────────────────────────────
#ifdef WITH_ARUCO
    auto aruco_dict = cv::aruco::getPredefinedDictionary(ARUCO_DICT_ID);
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params(
        new cv::aruco::DetectorParameters());

    // Must match picam.yaml calibration values exactly
    cv::Mat cam_matrix   = (cv::Mat_<double>(3, 3)
        << 509.765, 0, 325.861,
              0, 515.545, 239.576,
              0,       0,       1);
    cv::Mat dist_coeffs  = (cv::Mat_<double>(5, 1)
        << 0.232565, -0.490644, 0.002399, 0.004479, 0.30223);

    std::cout << "[slam_reader] ArUco ENABLED (DICT_4X4_50"
              << ", marker=" << MARKER_SIZE_M * 100.0f << " cm"
              << ", stamp=" << ARUCO_CORNER_RADIUS << "px)\n";

    // Load marker world poses for relocalization (localization mode only)
    std::map<int, Sophus::SE3f> marker_world_poses;
    if (mode == MODE_LOC) {
        marker_world_poses =
            load_marker_world_poses(output_dir + "/MarkerPositions.txt");
        if (marker_world_poses.empty())
            std::cerr << "[slam_reader] No markers loaded — "
                         "ArUco pose injection disabled.\n";
#ifdef WITH_POSE_INJECT
        else
            std::cout << "[slam_reader] Pose injection ENABLED "
                         "(WITH_POSE_INJECT).\n";
#else
        else
            std::cout << "[slam_reader] Pose injection DISABLED "
                         "(compile with -DWITH_POSE_INJECT after patching "
                         "ORB-SLAM3).\n";
#endif
    }
#endif  // WITH_ARUCO

    // ── Shared memory ────────────────────────────────────────────────────
    std::cout << "[slam_reader] Waiting for camera bridge on '"
              << SHM_NAME << "' ...\n";
    int shm_fd = -1;
    for (int attempt = 0; attempt < 300; ++attempt) {
        shm_fd = shm_open(SHM_NAME, O_RDONLY, 0);
        if (shm_fd >= 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (shm_fd < 0) {
        std::cerr << "[ERROR] Shared memory not found. Is bridge.py running?\n";
        return 1;
    }
    uint8_t *shm_ptr = static_cast<uint8_t *>(
        mmap(nullptr, SHM_SIZE, PROT_READ, MAP_SHARED, shm_fd, 0));
    if (shm_ptr == MAP_FAILED) { perror("mmap"); close(shm_fd); return 1; }
    std::cout << "[slam_reader] Shared memory mapped OK.\n";

    // ── ZMQ publisher ────────────────────────────────────────────────────
#ifdef WITH_ZMQ
    void *zmq_ctx = zmq_ctx_new();
    void *zmq_pub = zmq_socket(zmq_ctx, ZMQ_PUB);
    zmq_bind(zmq_pub, "tcp://*:5557");
    std::cout << "[slam_reader] ZMQ PUB on port 5557.\n";
#endif

    // ── ORB-SLAM3 init ───────────────────────────────────────────────────
    std::cout << "[slam_reader] Loading ORB-SLAM3 ...\n";
    ORB_SLAM3::System SLAM(vocab_path, runtime_config,
                           ORB_SLAM3::System::MONOCULAR, use_viewer);
    std::cout << "[slam_reader] SLAM Ready.\n";

    if (mode == MODE_LOC) {
        SLAM.ActivateLocalizationMode();
        std::cout << "[slam_reader] Localization mode activated.\n";
    }

    // Detect if ORB-SLAM3 silently creates a new map during mapping
    long unsigned int initial_map_id = 0;
    if (mode == MODE_MAP) {
        auto *init_map = SLAM.GetAtlas()->GetCurrentMap();
        if (init_map) {
            initial_map_id = init_map->GetId();
            std::cout << "[slam_reader] Mapping on map id=" << initial_map_id << "\n";
        }
    }

    // ── Per-run state ────────────────────────────────────────────────────
    uint64_t last_seq      = UINT64_MAX;
    uint64_t frames_tracked = 0;
    uint64_t frames_lost    = 0;
    auto     t_start       = std::chrono::steady_clock::now();

#ifdef WITH_ARUCO
    // Mapping: collect SE3 observations per marker
    std::map<int, std::vector<MarkerObsSE3>> marker_observations;

    // Localization: relocalization state machine
    //
    //  IDLE           → no action; waiting for a lost event with a known marker
    //  MAPPING_MODE   → ActivateLocalizationMode disabled; ORB-SLAM3 runs full
    //                   SLAM so it can add keyframes and rebuild local map context.
    //                   On every frame we also inject the ArUco-derived pose so
    //                   the tracker starts from the right position.
    //
    // Transitions:
    //   IDLE       → MAPPING_MODE  : first lost frame where a known marker is visible
    //   MAPPING_MODE → IDLE        : tracking recovers (re-enables loc mode), or timeout
    //
    enum class RelocState { IDLE, MAPPING_MODE };
    RelocState reloc_state  = RelocState::IDLE;
    int        reloc_frames = 0;
    int        consec_lost  = 0;
    // Stay in mapping mode for up to this many frames before giving up and retrying.
    // ~2 s at 30 fps — long enough for ORB-SLAM3 to relocalize and add keyframes.
    static constexpr int RELOC_MAX_FRAMES = 60;
#endif  // WITH_ARUCO

    // ════════════════════════════════════════════════════════════════════
    //  Tracking loop
    // ════════════════════════════════════════════════════════════════════
    while (!g_stop.load()) {

        // ── Read one frame from shared memory (double-read for consistency) ──
        uint64_t seq, seq2;
        double   ts;
        cv::Mat  frame(HEIGHT, WIDTH, CV_8UC3);
        do {
            seq  = read_u64(shm_ptr);
            ts   = read_f64(shm_ptr + 8);
            std::memcpy(frame.data, shm_ptr + HEADER_BYTES, FRAME_BYTES);
            seq2 = read_u64(shm_ptr);
        } while (seq != seq2);

        if (seq == last_seq) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        last_seq = seq;

        // ── ArUco detection ──────────────────────────────────────────────
#ifdef WITH_ARUCO
        std::vector<int>                            ids;
        std::vector<std::vector<cv::Point2f>>       corners, rejected;
        cv::aruco::detectMarkers(frame,
                                 cv::makePtr<cv::aruco::Dictionary>(aruco_dict),
                                 corners, ids, aruco_params, rejected);

        // Stamp corners at the SAME radius used during mapping.
        // Changing the radius changes the ORB descriptor and breaks BoW matching.
        if (!ids.empty())
            stamp_aruco_corners(frame, corners, ARUCO_CORNER_RADIUS);
#endif

        // ── Floor mask (suppress low-feature carpet region) ──────────────
        frame(cv::Rect(0, 300, WIDTH, HEIGHT - 300)).setTo(0);

        // ── BGR → RGB then track ─────────────────────────────────────────
        cv::Mat frame_rgb;
        cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);

        Sophus::SE3f Tcw          = SLAM.TrackMonocular(frame_rgb, ts);
        int          track_state  = SLAM.GetTrackingState();
        bool         ok           = (track_state == 2);   // 2 = Tracking::OK

        // ── Map integrity guard (mapping mode) ───────────────────────────
        if (mode == MODE_MAP) {
            auto *cur_map = SLAM.GetAtlas()->GetCurrentMap();
            if (cur_map && cur_map->GetId() != initial_map_id) {
                std::cout << "[slam_reader] WARNING: New map created (id="
                          << cur_map->GetId()
                          << ") — ORB-SLAM3 has reset. Stay on course.\n";
            }
        }

        // ── ArUco-assisted relocalization (localization mode only) ────────
        //
        // Strategy:
        //   When tracking is lost and we can see a known marker, switch ORB-SLAM3
        //   from localization mode to full SLAM mode (DeactivateLocalizationMode).
        //   In full SLAM mode the tracker can add new keyframes and rebuild the
        //   local map, giving it enough context to relocalize.
        //
        //   Simultaneously, compute the exact camera world pose from ArUco PnP
        //   and inject it via InjectPoseHint (requires ORB-SLAM3 patch).
        //   This seeds mLastFrame with the correct Tcw so that on the next frame
        //   TrackWithMotionModel starts from the right position rather than from
        //   an arbitrarily stale or zero pose.
        //
        //   Once tracking recovers, re-enable localization mode and return to IDLE.
        //   If RELOC_MAX_FRAMES elapses without recovery, return to loc mode and
        //   immediately retry (another IDLE→MAPPING_MODE transition will fire if
        //   the marker is still visible on the next lost frame).
#ifdef WITH_ARUCO
        if (mode == MODE_LOC) {

            if (!ok) {
                ++consec_lost;

                // Find the first visible marker with a known world pose.
                // "First" is fine — any known marker gives us a valid global pose.
                int best_idx = -1;
                for (size_t i = 0; i < ids.size(); ++i) {
                    if (marker_world_poses.count(ids[i])) {
                        best_idx = (int)i;
                        break;
                    }
                }

                // IDLE → MAPPING_MODE on the very first lost frame with a known marker.
                // (No 10-frame delay — every frame of delay is a frame closer to reset.)
                if (reloc_state == RelocState::IDLE && best_idx >= 0) {
                    SLAM.DeactivateLocalizationMode();
                    reloc_state  = RelocState::MAPPING_MODE;
                    reloc_frames = 0;
                    std::cout << "[slam_reader] ArUco reloc START — M"
                              << ids[best_idx]
                              << " visible, entered mapping mode\n";
                }

                if (reloc_state == RelocState::MAPPING_MODE) {
                    ++reloc_frames;

                    // Inject camera pose derived from the visible marker's SE3.
                    if (best_idx >= 0) {
                        std::vector<cv::Vec3d> rvecs, tvecs;
                        cv::aruco::estimatePoseSingleMarkers(
                            std::vector<std::vector<cv::Point2f>>{corners[(size_t)best_idx]},
                            MARKER_SIZE_M, cam_matrix, dist_coeffs, rvecs, tvecs);

                        // T_cm : marker in camera frame  (from PnP)
                        Eigen::Matrix3f R_cm = rvec_to_eigen(rvecs[0]);
                        Eigen::Vector3f t_cm((float)tvecs[0][0],
                                            (float)tvecs[0][1],
                                            (float)tvecs[0][2]);
                        Sophus::SE3f T_cm(R_cm, t_cm);

                        // T_wm : marker in world frame   (loaded from file)
                        const Sophus::SE3f &T_wm =
                            marker_world_poses.at(ids[(size_t)best_idx]);

                        // T_wc = T_wm * T_cm^{-1}   →   camera in world
                        // Tcw  = T_wc^{-1}           →   ORB-SLAM3 convention
                        Sophus::SE3f Tcw_hint = (T_wm * T_cm.inverse()).inverse();

#ifdef WITH_POSE_INJECT
                        // InjectPoseHint patches mLastFrame.pose and resets velocity
                        // so the next TrackMonocular call starts from ArUco pose.
                        // Requires adding InjectPoseHint to ORB-SLAM3 System.h/.cc
                        // (see comments at the bottom of this file).
                        bool injected = SLAM.InjectPoseHint(Tcw_hint);
                        if (injected)
                            std::cout << "[slam_reader] ArUco reloc: pose injected"
                                      << " from M" << ids[(size_t)best_idx]
                                      << " (frame " << reloc_frames << ")\n";
#else
                        // Without the patch, the ArUco pose is computed but not injected.
                        // The improved mapping-mode cycle still helps; add the patch for
                        // reliable single-frame recovery.
                        (void)Tcw_hint;
#endif  // WITH_POSE_INJECT
                    }

                    // Timeout: return to localization mode and retry on next lost frame.
                    if (reloc_frames >= RELOC_MAX_FRAMES) {
                        SLAM.ActivateLocalizationMode();
                        reloc_state  = RelocState::IDLE;
                        reloc_frames = 0;
                        std::cout << "[slam_reader] ArUco reloc TIMEOUT ("
                                  << RELOC_MAX_FRAMES << " frames) — retrying\n";
                    }
                }

            } else {
                // Tracking is OK — clean up state machine.
                if (reloc_state == RelocState::MAPPING_MODE) {
                    SLAM.ActivateLocalizationMode();
                    std::cout << "[slam_reader] ArUco reloc RECOVERED in "
                              << reloc_frames << " frames  (consec_lost="
                              << consec_lost << ")\n";
                }
                reloc_state  = RelocState::IDLE;
                reloc_frames = 0;
                consec_lost  = 0;
            }
        }
#endif  // WITH_ARUCO

        // ── ZMQ broadcast ────────────────────────────────────────────────
#ifdef WITH_ZMQ
        {
            std::string json = pose_to_json(Tcw, ts, seq, ok);
            zmq_send(zmq_pub, json.c_str(), json.size(), ZMQ_NOBLOCK);
        }
#endif

        // ── Collect marker SE3 observations (mapping mode, good tracking) ─
#ifdef WITH_ARUCO
        if (ok && mode == MODE_MAP && !ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                corners, MARKER_SIZE_M, cam_matrix, dist_coeffs, rvecs, tvecs);
            Sophus::SE3f Twc = Tcw.inverse();

            for (size_t i = 0; i < ids.size(); ++i) {
                Eigen::Matrix3f R_cm = rvec_to_eigen(rvecs[i]);
                Eigen::Vector3f t_cm((float)tvecs[i][0],
                                    (float)tvecs[i][1],
                                    (float)tvecs[i][2]);
                Sophus::SE3f T_cm(R_cm, t_cm);
                // T_wm = T_wc * T_cm  →  marker pose in world frame
                Sophus::SE3f T_wm = Twc * T_cm;

                MarkerObsSE3 obs;
                obs.pos_world = T_wm.translation();
                obs.rot_world = Eigen::Quaternionf(T_wm.rotationMatrix());
                marker_observations[ids[i]].push_back(obs);
            }
        }
#endif

        // ── Console logging ──────────────────────────────────────────────
        if (ok) {
            ++frames_tracked;
            Eigen::Vector3f t = cam_position(Tcw);
            std::cout << std::fixed << std::setprecision(4)
                      << "[SLAM] seq=" << seq
                      << "  ts="  << ts
                      << "  x="   << t.x()
                      << "  y="   << t.y()
                      << "  z="   << t.z()
#ifdef WITH_ARUCO
                      << "  aruco=" << ids.size()
#endif
                      << "\n";
        } else {
            ++frames_lost;
            std::cout << "[SLAM] seq=" << seq << "  TRACKING LOST\n";
        }

        // Stats line every 5 seconds
        auto   now     = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - t_start).count();
        if (elapsed > 5.0) {
            double fps = (frames_tracked + frames_lost) / elapsed;
            std::cout << "[slam_reader] " << frames_tracked << " tracked  "
                      << frames_lost  << " lost  "
                      << std::setprecision(1) << fps << " fps\n";
            frames_tracked = frames_lost = 0;
            t_start = now;
        }
    }  // end tracking loop

    // ════════════════════════════════════════════════════════════════════
    //  Clean shutdown
    // ════════════════════════════════════════════════════════════════════
    std::cout << "\n==================================================\n";
    std::cout << "[slam_reader] Shutting down ...\n";

    if (mode == MODE_MAP) {
        std::cout << "[slam_reader] Saving KeyFrameTrajectory.txt ...\n";
        SLAM.SaveKeyFrameTrajectoryTUM(
            (output_dir + "/KeyFrameTrajectory.txt").c_str());
        save_map_points(SLAM, output_dir + "/MapPoints.txt");
#ifdef WITH_ARUCO
        save_marker_positions(marker_observations, output_dir + "/MarkerPositions.txt");
#endif
        std::string cmd =
            "python3 ../map_builder.py"
            " --points "  + output_dir + "/MapPoints.txt"
            " --traj "    + output_dir + "/KeyFrameTrajectory.txt"
            " --markers " + output_dir + "/MarkerPositions.txt"
            " --out "     + output_dir;
        std::cout << "[slam_reader] $ " << cmd << "\n";
        int ret = std::system(cmd.c_str());
        if (ret == 0) std::cout << "[slam_reader] map_builder.py finished OK.\n";
        else          std::cerr << "[slam_reader] map_builder.py failed (exit "
                                << ret << ").\n";
        std::cout << "[slam_reader] Calling Shutdown (will save atlas) ...\n";
    }

    // Watchdog: force-exit if SLAM.Shutdown() hangs > 30 s
    std::thread watchdog([]() {
        std::this_thread::sleep_for(std::chrono::seconds(30));
        std::cout << "[slam_reader] Watchdog fired — force exiting.\n";
        std::quick_exit(0);
    });
    watchdog.detach();
    SLAM.Shutdown();

    munmap(shm_ptr, SHM_SIZE);
    close(shm_fd);
#ifdef WITH_ZMQ
    zmq_close(zmq_pub);
    zmq_ctx_destroy(zmq_ctx);
#endif

    std::cout << "      SHUTDOWN COMPLETE\n";
    std::cout << "==================================================\n";
    std::cout << "[slam_reader] Force exiting to prevent hang.\n";
    std::quick_exit(0);
}

// ══════════════════════════════════════════════════════════════════════════
//  ORB-SLAM3 patch required for -DWITH_POSE_INJECT
//  Apply these two changes, then recompile ORB-SLAM3.
//
//  ── 1. ORB_SLAM3/include/System.h ────────────────────────────────────────
//  Add inside the public section (e.g. after ActivateLocalizationMode):
//
//      // Seed the tracker with a known pose when tracking is lost.
//      // Call immediately after TrackMonocular returns a lost state.
//      // Returns true if the hint was accepted (state was RECENTLY_LOST or LOST).
//      // Requires -DWITH_POSE_INJECT in slam_reader build.
//      bool InjectPoseHint(const Sophus::SE3f& Tcw);
//
//  ── 2. ORB_SLAM3/src/System.cc ───────────────────────────────────────────
//  Add at the end of the file (before the closing namespace brace):
//
//      bool System::InjectPoseHint(const Sophus::SE3f& Tcw) {
//          if (!mpTracker) return false;
//          // Only inject when the tracker is actually lost — injecting
//          // while tracking is OK would corrupt the motion model.
//          int state = mpTracker->mState;
//          if (state != Tracking::RECENTLY_LOST && state != Tracking::LOST)
//              return false;
//          // Overwrite last frame pose so TrackWithMotionModel starts from
//          // the ArUco-derived position on the next TrackMonocular call.
//          mpTracker->mLastFrame.SetPose(Tcw);
//          // Zero velocity: predict "same position next frame".
//          // This ensures TrackWithMotionModel uses our injected pose as-is
//          // rather than extrapolating from a stale velocity estimate.
//          mpTracker->mVelocity = Sophus::SE3f();
//          // Mark as OK so the next TrackMonocular call uses the normal
//          // tracking path (TrackWithMotionModel / TrackReferenceKeyFrame)
//          // rather than the relocalization path.
//          mpTracker->mState = Tracking::OK;
//          return true;
//      }
//
//  ── Build ────────────────────────────────────────────────────────────────
//  After patching:
//      cd ~/autonomous_car/ORB_SLAM3
//      make -j$(nproc)
//
//  Then rebuild slam_reader with the new flag:
//      cd ~/autonomous_car/scripts/build
//      cmake .. -DWITH_ARUCO=ON -DWITH_ZMQ=ON -DWITH_POSE_INJECT=ON
//      make -j$(nproc)
//
//  ── Remap after all changes ───────────────────────────────────────────────
//  The new MarkerPositions.txt format includes quaternion.
//  Run a fresh mapping session — the old room_map.osa and MarkerPositions.txt
//  are still valid for localization (the loader falls back gracefully),
//  but pose injection will be inaccurate without rotation data.
// ══════════════════════════════════════════════════════════════════════════
