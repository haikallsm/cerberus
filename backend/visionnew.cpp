#include <iostream>
#include <vector>
#include <deque>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <memory>
#include <onnxruntime_cxx_api.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/msg/status_text.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include <microhttpd.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/sdp/sdp.h>
#include <gst/webrtc/webrtc.h>

#define GST_USE_UNSTABLE_API 1

#include <nlohmann/json.hpp>
#include <microhttpd.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/sdp/sdp.h>
#include <gst/webrtc/webrtc.h>

using json = nlohmann::json;

// ... [Biarkan semua #include rclcpp dan OpenCV Anda di sini] ...

// ============================================================
// PATH MODEL
// ============================================================
const std::string PALM_MODEL = "/home/hawkeye-expro/ros2_ws/src/hawkeyev2/models/palm_detection_mediapipe_2023feb.onnx";
const std::string HANDPOSE_MODEL = "/home/hawkeye-expro/ros2_ws/src/hawkeyev2/models/handpose_estimation_mediapipe_2023feb.onnx";
// Model deteksi orang (YOLOv8n), mengikuti vision.py (ONNX_MODEL_PATH)
const std::string PERSON_MODEL = "/home/hawkeye-expro/ros2_ws/src/hawkeyev2/models/yolov8n.onnx";
// const std::string PERSON_MODEL = "/home/hawkeye-expro/ros2_ws/src/hawkeyev2/models/yolov8n480.onnx";
// ============================================================
// PARAMETER
// ============================================================
constexpr float PALM_SCORE_THRESHOLD = 0.30f;
constexpr float PALM_NMS_THRESHOLD   = 0.30f;
// PENTING (fix "objek bukan tangan terbaca sebagai tangan"):
// Sebelumnya cuma 0.3 -- terlalu longgar. Dokumentasi resmi
// OpenCV Zoo untuk model handpose_estimation_mediapipe ini
// pakai default confThreshold = 0.8. Confidence rendah artinya
// modelnya sendiri ragu itu tangan (bisa jadi kepalan tangan
// yang sudut anehnya bikin bingung, atau memang bukan tangan
// sama sekali seperti kepalan baju/benda berwarna kulit).
// Dinaikkan ke 0.7 sebagai titik tengah: cukup ketat untuk
// menyaring false-positive, tapi masih memberi sedikit toleransi
// dibanding 0.8 penuh untuk kondisi jarak jauh/rotasi ekstrem.
// Kalau masih ada objek yang salah terbaca, naikkan lagi ke 0.8.
constexpr float HAND_CONF_THRESHOLD  = 0.7f;
// Threshold lebih longgar khusus untuk MENCARI tangan jarak jauh
// (dipakai saat tiled/multi-scale search, bukan saat sudah tracking,
//  supaya tidak terlalu banyak false-positive di frame biasa).
// Palm detector di sini cuma penyaring kasar tahap awal -- kandidat
// hasilnya tetap harus lolos HAND_CONF_THRESHOLD yang jauh lebih
// ketat di atas sebelum dianggap tangan sungguhan.
constexpr float PALM_SCORE_THRESHOLD_SEARCH = 0.20f;
// Jumlah frame untuk analisis gerakan melambai
constexpr int WAVE_HISTORY_SIZE = 20;
// Minimal perpindahan wrist relatif terhadap ukuran tangan
constexpr float WAVE_MOVEMENT_THRESHOLD = 0.20f;
// ============================================================
// PARAMETER DETEKSI ORANG + TRACKING MULTI-ORANG
// (nilai mengikuti vision.py persis)
// ============================================================
constexpr float ONNX_CONF_THRESHOLD    = 0.4f;
constexpr float ONNX_NMS_THRESHOLD     = 0.45f;
constexpr int   ONNX_PERSON_CLASS_ID   = 0;
constexpr int   ONNX_INPUT_SIZE_FALLBACK = 360;
constexpr float TRACK_MATCH_MAX_DIST   = 90.0f;
constexpr int   TRACK_MAX_MISS         = 5;
constexpr int   WAVE_MISS_RESET_FRAMES = 10;
constexpr int   DETECTION_INTERVAL     = 5;
constexpr float ARM_ROI_PADDING_RATIO  = 0.15f;

// ============================================================
// STRUCT
// ============================================================
struct PalmDetection {
    cv::Rect2f bbox;
    std::vector<cv::Point2f> landmarks;
    float score;
};

struct HandResult {
    std::vector<cv::Point3f> landmarks;
    cv::Rect2f bbox;
    float confidence;
};

// ============================================================
// GENERATE 2016 PALM ANCHORS
// Sesuai struktur anchor MP-PalmDet OpenCV Zoo
// ============================================================
std::vector<cv::Point2f> generateAnchors() {
    std::vector<cv::Point2f> anchors;
    anchors.reserve(2016);
    // --------------------------------------------------------
    // Feature map 24x24
    // 576 cells × 2 anchors = 1152
    // --------------------------------------------------------
    for (int y = 0; y < 24; ++y) {
        for (int x = 0; x < 24; ++x) {
            float cx = (x + 0.5f) / 24.0f;
            float cy = (y + 0.5f) / 24.0f;
            anchors.emplace_back(cx, cy);
            anchors.emplace_back(cx, cy);
        }
    }
    // --------------------------------------------------------
    // Feature map 12x12
    // 144 cells × 6 anchors = 864
    // --------------------------------------------------------
    for (int y = 0; y < 12; ++y) {
        for (int x = 0; x < 12; ++x) {
            float cx = (x + 0.5f) / 12.0f;
            float cy = (y + 0.5f) / 12.0f;
            for (int i = 0; i < 6; ++i) {
                anchors.emplace_back(cx, cy);
            }
        }
    }
    return anchors;
}

// ============================================================
// NHWC BLOB
// Model MediaPipe ini menggunakan:
// [1, H, W, 3]
// ============================================================
cv::Mat makeNHWCBlob( const cv::Mat& image, int width, int height) {
    // Skip resize kalau ukurannya sudah pas (letterbox biasanya
    // sudah menghasilkan ukuran exact INPUT_SIZE x INPUT_SIZE)
    cv::Mat resized;
    if (image.cols != width || image.rows != height) {
        cv::resize(image, resized, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
    } else {
        resized = image;
    }
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    // convertTo pakai implementasi vectorized/SIMD OpenCV,
    // jauh lebih cepat dibanding loop manual per-piksel
    // dengan .at<>() (terutama penting karena fungsi ini
    // dipanggil berkali-kali per frame saat tiled search).
    cv::Mat floatImg;
    rgb.convertTo(floatImg, CV_32FC3, 1.0 / 255.0);
    // Layout memori cv::Mat (H, W, 3channel float, kontigu)
    // SAMA PERSIS dengan NHWC batch=1, jadi cukup memcpy
    // sekali tanpa loop per-piksel sama sekali.
    int sizes[] = {1, height, width, 3};
    cv::Mat blob(4, sizes, CV_32F);
    std::memcpy(blob.data, floatImg.data, floatImg.total() * floatImg.elemSize());
    return blob;
}

// ============================================================
// CAMERA RECONNECT
//
// Port dari vision.py: class CameraReconnect. Logikanya sama
// persis -- baca kamera, hitung kegagalan berturut-turut, dan
// coba reconnect dengan exponential backoff kalau kamera lepas
// koneksi (mis. kabel USB longgar), termasuk mencoba beberapa
// index kandidat kalau device index-nya berubah setelah reconnect.
// ============================================================
class CameraReconnect {
private:
    cv::VideoCapture cap;
    int width;
    int height;
    int fps;
    int maxReadFailures;
    double backoffBase;
    double backoffMax;
    std::vector<int> candidateIndices;
    int cameraIndex;
    int consecutiveFailures = 0;
    int reconnectAttempts   = 0;
    std::chrono::steady_clock::time_point lastReconnectAttempt = std::chrono::steady_clock::now() - std::chrono::hours(1);
    bool openIndex(int index) {
        if (cap.isOpened()) {
            cap.release();
        }
        cap.open(index, cv::CAP_V4L2);
        if (!cap.isOpened()) {
            return false;
        }
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        // Kurangi latency stale frame pasca-reconnect
        // cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        cap.set(cv::CAP_PROP_FPS, fps);
        cameraIndex = index;
        consecutiveFailures = 0;
        reconnectAttempts = 0;
        std::cout << "[CAMERA] Terhubung ulang di index " << index << "\n";
        return true;
    }
public:
    CameraReconnect(int cameraIndex_, int width_, int height_, int maxReadFailures_ = 15, double backoffBase_ = 0.5, double backoffMax_ = 10.0, std::vector<int> candidateIndices_ = {}) : width(width_), height(height_), maxReadFailures(maxReadFailures_), backoffBase(backoffBase_), backoffMax(backoffMax_), cameraIndex(cameraIndex_) {
        candidateIndices = candidateIndices_.empty() ? std::vector<int>{cameraIndex_, 0, 1, 2} : candidateIndices_;
        if (!openIndex(cameraIndex_)) {
            std::cerr << "[CAMERA] Index " << cameraIndex_ << " gagal dibuka saat init.\n";
        }
    }
    bool isOpened() const {
        return cap.isOpened();
    }
    bool read(cv::Mat& frame) {
        if (!cap.isOpened()) {
            consecutiveFailures++;
            return false;
        }
        cap >> frame;
        if (frame.empty()) {
            consecutiveFailures++;
            return false;
        }
        consecutiveFailures = 0;
        return true;
    }
    bool needsReconnect() const {
        return consecutiveFailures >= maxReadFailures;
    }
    bool tryReconnect() {
        auto now = std::chrono::steady_clock::now();
        double backoff = std::min(backoffBase * std::pow(2.0, reconnectAttempts), backoffMax);
        double elapsed = std::chrono::duration<double>(now - lastReconnectAttempt).count();
        if (elapsed < backoff) {
            return false;
        }
        lastReconnectAttempt = now;
        reconnectAttempts++;
        std::cout << "[CAMERA] Reconnect attempt #" << reconnectAttempts << "...\n";
        for (int idx : candidateIndices) {
            if (openIndex(idx)) {
                return true;
            }
        }
        std::cerr << "[CAMERA] Reconnect gagal di semua kandidat index, " << "retry dalam ~" << backoff << "s\n";
        return false;
    }

    void release() {
        if (cap.isOpened()) {
            cap.release();
        }
    }
};

// ============================================================
// PALM DETECTOR
// ============================================================
class PalmDetector {
private:
    Ort::Session session{nullptr};
    Ort::SessionOptions sessionOptions;
    std::string inputName;
    std::vector<std::string> outputNames;
    std::vector<cv::Point2f> anchors;
    const int INPUT_SIZE = 192;
public:
    PalmDetector(Ort::Env& env, const std::string& modelPath) {
        std::cout << "Loading palm model with ONNX Runtime...\n";
        // PENTING untuk FPS: cv::dnn (backend lama) jauh lebih
        // lambat dibanding ONNX Runtime di CPU/ARM (Raspberry Pi),
        // karena cv::dnn tidak punya optimisasi kernel setara
        // XNNPACK/ORT. Ini salah satu penyebab utama FPS kalah
        // dibanding versi Python (MediaPipe pakai TFLite+XNNPACK).
        // sessionOptions.SetIntraOpNumThreads(4);
        sessionOptions.SetInterOpNumThreads(1);
        sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session = Ort::Session(env, modelPath.c_str(), sessionOptions);
        Ort::AllocatorWithDefaultOptions allocator;
        {
            auto name = session.GetInputNameAllocated(0, allocator);
            inputName = name.get();
        }
        size_t outputCount = session.GetOutputCount();
        for (size_t i = 0; i < outputCount; ++i) {
            auto name = session.GetOutputNameAllocated(i, allocator);
            outputNames.push_back(name.get());
        }
        anchors = generateAnchors();
        if (anchors.size() != 2016) {
            throw std::runtime_error("Jumlah anchor tidak 2016!");
        }
        std::cout << "Palm model loaded successfully (ONNX Runtime)\n";
        std::cout << "Input : " << inputName << "\n";
        std::cout << "Outputs: " << outputNames.size() << "\n";
    }

    std::vector<PalmDetection> detect(const cv::Mat& frame, float scoreThreshold = PALM_SCORE_THRESHOLD) {
        // ----------------------------------------------------
        // Letterbox seperti implementasi OpenCV Zoo
        // ----------------------------------------------------
        float ratio = std::min(static_cast<float>(INPUT_SIZE) / frame.cols, static_cast<float>(INPUT_SIZE) / frame.rows);
        cv::Size resizedSize(static_cast<int>(frame.cols * ratio), static_cast<int>(frame.rows * ratio));
        cv::Mat resized;
        cv::resize(frame, resized, resizedSize);
        int padW = INPUT_SIZE - resized.cols;
        int padH = INPUT_SIZE - resized.rows;
        int padLeft = padW / 2;
        int padTop = padH / 2;
        cv::Mat padded;
        cv::copyMakeBorder(resized, padded, padTop, padH - padTop, padLeft, padW - padLeft, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::Mat input = makeNHWCBlob(padded, INPUT_SIZE, INPUT_SIZE);
        // ----------------------------------------------------
        // ONNX Runtime inference
        // ----------------------------------------------------
        std::array<int64_t, 4> inputShape = {1, INPUT_SIZE, INPUT_SIZE, 3};
        size_t inputTensorSize = static_cast<size_t>(INPUT_SIZE) * INPUT_SIZE * 3;
        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(memoryInfo, reinterpret_cast<float*>(input.data), inputTensorSize, inputShape.data(), inputShape.size());
        const char* inputNamePtr = inputName.c_str();
        std::vector<const char*> outputNamePtrs;
        for (const auto& name : outputNames) {
            outputNamePtrs.push_back(name.c_str());
        }
        auto outputs = session.Run(Ort::RunOptions{nullptr}, &inputNamePtr, &inputTensor, 1, outputNamePtrs.data(), outputNamePtrs.size());
        if (outputs.size() < 2) {
            return {};
        }
        // Identifikasi output berdasarkan bentuk (shape), bukan
        // urutan index -- lebih aman karena urutan output ONNX
        // Runtime tidak selalu sama dengan urutan di cv::dnn.
        // Output box: [1, 2016, 18], output score: [1, 2016, 1]
        const float* boxesData  = nullptr;
        const float* scoresData = nullptr;
        for (auto& out : outputs) {
            auto shape = out.GetTensorTypeAndShapeInfo().GetShape();
            if (shape.empty())
                continue;
            int64_t lastDim = shape.back();
            if (lastDim == 18) {
                boxesData = out.GetTensorData<float>();
            } else if (lastDim == 1) {
                scoresData = out.GetTensorData<float>();
            }
        }
        if (!boxesData || !scoresData) {
            return {};
        }
        std::vector<cv::Rect> nmsBoxes;
        std::vector<float> confidenceScores;
        std::vector<PalmDetection> candidates;
        float scale = static_cast<float>(std::max(frame.cols, frame.rows));
        // ----------------------------------------------------
        // BUG FIX PENTING:
        //
        // Sebelumnya kode ini melakukan:
        //   x = (...)*scale; x -= padLeft; x /= ratio;
        // Padahal formula resmi OpenCV Zoo (mp_palmdet.py) itu:
        //   x = (...)*scale - (padLeft / ratio)
        // (pad_bias dikonversi ke satuan pixel ASLI sekali saja,
        //  BUKAN dikurangkan dulu baru seluruh nilai dibagi ratio
        //  lagi). Karena (...)* scale itu SUDAH otomatis setara
        //  dengan (posisi di ruang 192px) / ratio (turunannya:
        //  scale = max(w,h) = 192/ratio), pembagian ratio kedua
        //  kali di atas membuat koordinat kena bagi ratio DUA
        //  KALI -> bounding box & 7 landmark palm meleset jauh
        //  dari posisi tangan sebenarnya. Ini kemungkinan besar
        //  penyebab utama landmark/deteksi yang salah.
        // ----------------------------------------------------
        float padBiasX = static_cast<float>(padLeft) / ratio;
        float padBiasY = static_cast<float>(padTop) / ratio;
        // ----------------------------------------------------
        // Decode 2016 detections
        // ----------------------------------------------------
        for (int i = 0; i < 2016; ++i) {
            float rawScore = scoresData[i];
            float score = 1.0f / (1.0f + std::exp(-rawScore));
            if (score < scoreThreshold)
                continue;
            cv::Point2f anchor = anchors[i];
            const float* boxRow = boxesData + i * 18;
            float cx = boxRow[0] / INPUT_SIZE;
            float cy = boxRow[1] / INPUT_SIZE;
            float w = boxRow[2] / INPUT_SIZE;
            float h = boxRow[3] / INPUT_SIZE;
            float x1 = (cx - w / 2.0f + anchor.x) * scale - padBiasX;
            float y1 = (cy - h / 2.0f + anchor.y) * scale - padBiasY;
            float x2 = (cx + w / 2.0f + anchor.x) * scale - padBiasX;
            float y2 = (cy + h / 2.0f + anchor.y) * scale - padBiasY;
            cv::Rect2f box(x1, y1, x2 - x1, y2 - y1);
            // ------------------------------------------------
            // 7 palm landmarks
            // ------------------------------------------------
            std::vector<cv::Point2f> palmLandmarks;
            for (int j = 0; j < 7; ++j) {
                float lx = boxRow[4 + j * 2] / INPUT_SIZE;
                float ly = boxRow[4 + j * 2 + 1] / INPUT_SIZE;
                lx = (lx + anchor.x) * scale - padBiasX;
                ly = (ly + anchor.y) * scale - padBiasY;
                palmLandmarks.emplace_back(lx, ly);
            }
            candidates.push_back({box, palmLandmarks, score});
            nmsBoxes.emplace_back(static_cast<int>(box.x), static_cast<int>(box.y), static_cast<int>(box.width), static_cast<int>(box.height));
            confidenceScores.push_back(score);
        }
        // ----------------------------------------------------
        // NMS
        // ----------------------------------------------------
        std::vector<int> indices;
        cv::dnn::NMSBoxes(nmsBoxes, confidenceScores, scoreThreshold, PALM_NMS_THRESHOLD, indices);
        std::vector<PalmDetection> results;
        for (int index : indices) {
            if (index >= 0 && index < static_cast<int>(candidates.size())) {
                results.push_back(candidates[index]);
            }
        }
        return results;
    }

    // ========================================================
    // MULTI-SCALE / TILED DETECTION
    //
    // Palm detector MediaPipe input-nya cuma 192x192. Kalau
    // tangan berada ~3 meter dari kamera, ukurannya di frame
    // penuh jadi sangat kecil (cuma beberapa persen lebar
    // frame) -> setelah di-downscale ke 192px, tangan itu
    // tinggal beberapa pixel saja, sehingga model gagal
    // mendeteksinya walau threshold sudah diturunkan.
    //
    // Solusinya: pecah frame jadi beberapa tile yang saling
    // overlap, lalu jalankan palm detector di tiap tile
    // (bukan di frame penuh). Tangan yang jauh & kecil di
    // frame penuh akan terlihat jauh LEBIH BESAR relatif di
    // dalam satu tile, sehingga jauh lebih mudah terdeteksi.
    //
    // Deteksi dari full-frame + semua tile digabung lalu
    // di-NMS ulang supaya tidak dobel.
    // ========================================================
    std::vector<PalmDetection> detectMultiScale(const cv::Mat& frame) {
        std::vector<PalmDetection> all;
        // ----------------------------------------------------
        // Pass 1: full-frame (menangkap tangan yang besar/dekat)
        // ----------------------------------------------------
        std::vector<PalmDetection> fullFrameDet = detect(frame, PALM_SCORE_THRESHOLD);
        all.insert(all.end(), fullFrameDet.begin(), fullFrameDet.end());
        // ----------------------------------------------------
        // Pass 2: tiled search (menangkap tangan kecil/jauh)
        // ----------------------------------------------------
        const int tilesX = 2;
        const int tilesY = 2;
        const float overlap = 0.25f; // 25% overlap antar tile
        int tileW = static_cast<int>(frame.cols / (tilesX - (tilesX - 1) * overlap));
        int tileH = static_cast<int>(frame.rows / (tilesY - (tilesY - 1) * overlap));
        tileW = std::max(tileW, 32);
        tileH = std::max(tileH, 32);
        int strideX = std::max(1, static_cast<int>(tileW * (1.0f - overlap)));
        int strideY = std::max(1, static_cast<int>(tileH * (1.0f - overlap)));
        for (int ty = 0; ty < tilesY; ++ty) {
            for (int tx = 0; tx < tilesX; ++tx) {
                int x = tx * strideX;
                int y = ty * strideY;
                x = std::min(x, std::max(0, frame.cols - tileW));
                y = std::min(y, std::max(0, frame.rows - tileH));
                int w = std::min(tileW, frame.cols - x);
                int h = std::min(tileH, frame.rows - y);
                if (w < 32 || h < 32)
                    continue;
                cv::Rect tileRect(x, y, w, h);
                cv::Mat tile = frame(tileRect);
                std::vector<PalmDetection> tileDet = detect(tile, PALM_SCORE_THRESHOLD_SEARCH);
                for (auto& d : tileDet) {
                    // Geser koordinat tile kembali ke koordinat frame penuh
                    d.bbox.x += static_cast<float>(tileRect.x);
                    d.bbox.y += static_cast<float>(tileRect.y);
                    for (auto& lm : d.landmarks) {
                        lm.x += static_cast<float>(tileRect.x);
                        lm.y += static_cast<float>(tileRect.y);
                    }
                    all.push_back(d);
                }
            }
        }
        if (all.empty())
            return all;
        // ----------------------------------------------------
        // NMS gabungan supaya deteksi dari tile yang overlap
        // tidak dihitung dobel
        // ----------------------------------------------------
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        for (const auto& d : all) {
            boxes.emplace_back(static_cast<int>(d.bbox.x), static_cast<int>(d.bbox.y), static_cast<int>(d.bbox.width), static_cast<int>(d.bbox.height));
            scores.push_back(d.score);
        }
        std::vector<int> keep;
        cv::dnn::NMSBoxes(boxes, scores, PALM_SCORE_THRESHOLD_SEARCH, PALM_NMS_THRESHOLD, keep);
        std::vector<PalmDetection> merged;
        for (int idx : keep) {
            merged.push_back(all[idx]);
        }
        // Urutkan dari skor tertinggi supaya palms[0] = deteksi terbaik
        std::sort(merged.begin(), merged.end(), [](const PalmDetection& a, const PalmDetection& b){return a.score > b.score;});
        return merged;
    }
};

// ============================================================
// HAND POSE
// ============================================================
class HandPose {
private:
    Ort::Session session{nullptr};
    Ort::SessionOptions sessionOptions;
    const int INPUT_SIZE = 224;
    std::string inputName;
    std::vector<std::string> outputNames;
public:
    HandPose(Ort::Env& env, const std::string& modelPath) {
        std::cout << "Loading hand pose model with ONNX Runtime...\n";
        // Dinaikkan dari 2 -> 4 supaya memanfaatkan semua core
        // CPU (mis. Raspberry Pi 5 = quad-core). Palm detector
        // dan hand pose berjalan bergantian (bukan bersamaan)
        // di loop utama, jadi aman memakai semua core untuk
        // masing-masing tanpa saling rebutan.
        // sessionOptions.SetIntraOpNumThreads(4);
        sessionOptions.SetInterOpNumThreads(1);
        sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session = Ort::Session(env, modelPath.c_str(), sessionOptions);
        // =====================================================
        // INPUT NAME
        // =====================================================
        Ort::AllocatorWithDefaultOptions allocator;
        {
            auto name = session.GetInputNameAllocated(0, allocator);
            inputName = name.get();
        }
        // =====================================================
        // OUTPUT NAMES
        // =====================================================
        size_t outputCount = session.GetOutputCount();
        for (size_t i = 0; i < outputCount; ++i) {
            auto name = session.GetOutputNameAllocated(i, allocator);
            outputNames.push_back(name.get());
        }
        std::cout << "Hand pose model loaded successfully\n";
        std::cout << "Input : " << inputName << "\n";
        std::cout << "Outputs: " << outputNames.size() << "\n";
    }


    HandResult estimate(const cv::Mat& frame, const PalmDetection& palm) {
        // =====================================================
        // 1. AMBIL PALM BBOX
        // =====================================================
        cv::Point2f p1 = palm.bbox.tl();
        cv::Point2f p2 = palm.bbox.br();
        float palmWidth = palm.bbox.width;
        float palmHeight = palm.bbox.height;
        if (palmWidth <= 5 || palmHeight <= 5) {
            return {};
        }
        cv::Point2f center = (p1 + p2) * 0.5f;
        // =====================================================
        // 1B. HITUNG SUDUT ROTASI TANGAN
        //
        // Palm detector memberi 7 keypoint. Berdasarkan urutan
        // OpenCV Zoo MP-PalmDet: index 0 = wrist, index 2 =
        // middle_finger_mcp. Vektor wrist->middle_mcp menunjuk
        // arah jari.
        //
        // Tanpa koreksi rotasi ini, saat tangan MELAMBAI (yang
        // artinya tangan berputar/miring terus-menerus), ROI
        // crop akan selalu diasumsikan "jari menghadap atas"
        // padahal tidak -> landmark hasil model jadi meleset
        // dari telapak tangan yang sebenarnya.
        // =====================================================
        double angleDeg = 0.0;
        if (palm.landmarks.size() >= 3) {
            const cv::Point2f& wristPt  = palm.landmarks[0];
            const cv::Point2f& midMcpPt = palm.landmarks[2];
            double radians = CV_PI / 2.0 - std::atan2(-(midMcpPt.y - wristPt.y), (midMcpPt.x - wristPt.x));
            // Normalisasi ke rentang [-pi, pi]
            radians -= 2.0 * CV_PI * std::floor((radians + CV_PI) / (2.0 * CV_PI));
            angleDeg = radians * 180.0 / CV_PI;
        }
        // =====================================================
        // 2. BUAT ROI YANG LEBIH BESAR + AMBIL LANGSUNG
        //    DARI FRAME ASLI DENGAN SATU warpAffine
        //
        // PENTING (fix bug "tangan dekat tidak terbaca"):
        // Versi sebelumnya me-rotate SELURUH frame lebih dulu
        // (warpAffine ukuran penuh) lalu meng-crop hasilnya
        // dengan BORDER_CONSTANT (hitam). Saat tangan sangat
        // DEKAT, ROI yang diperbesar (size*3.0) jadi sangat
        // besar relatif terhadap frame, sehingga area hitam
        // hasil rotasi ikut masuk ke ROI dalam porsi besar dan
        // menjatuhkan confidence model di bawah threshold.
        //
        // Solusinya: satu warpAffine yang langsung crop + rotate
        // + translate dari frame ASLI ke kanvas ROI (ukuran
        // `roiSize`), pakai BORDER_REPLICATE (bukan hitam) untuk
        // bagian yang jatuh di luar frame. Ini juga otomatis
        // menghasilkan kanvas yang SUDAH persegi, jadi tidak
        // perlu lagi langkah "SQUARE" terpisah, dan jauh lebih
        // murah secara komputasi karena warpAffine hanya
        // menghasilkan piksel sebesar ROI, bukan seluruh frame.
        // =====================================================
        float baseSize = std::max(palmWidth, palmHeight);
        float shiftUp = 0.4f * baseSize;
        float roiSizeF = baseSize * 3.0f;
        int side = std::max(1, static_cast<int>(std::round(roiSizeF)));
        // Matrix rotasi terhadap titik pivot = center bbox palm
        // (di frame ASLI)
        cv::Mat rotMat = cv::getRotationMatrix2D(center, angleDeg, 1.0);
        // Geser hasil rotasi supaya titik ROI (center digeser ke
        // atas sejauh shiftUp) jatuh persis di tengah kanvas
        // ROI berukuran side x side.
        rotMat.at<double>(0, 2) += (side / 2.0) - center.x;
        rotMat.at<double>(1, 2) += (side / 2.0) - (center.y - shiftUp);
        cv::Mat invRotMat;
        cv::invertAffineTransform(rotMat, invRotMat);
        cv::Mat square;
        cv::warpAffine(frame, square, rotMat, cv::Size(side, side), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
        if (square.empty())
            return {};
        // Kanvas `square` di atas SUDAH persegi & sudah berisi
        // ROI yang benar (crop + rotate + shift + enlarge dalam
        // satu langkah), jadi offsetX/offsetY = 0 untuk keperluan
        // mapping landmark di bawah.
        const int offsetX = 0;
        const int offsetY = 0;
        const int x1 = 0;
        const int y1 = 0;
        // =====================================================
        // 5. RESIZE 224x224
        //
        // Untuk tangan yang jauh (ROI crop asli lebih kecil
        // dari 224x224), pakai INTER_CUBIC karena ini upscaling
        // (INTER_AREA cocok untuk downscaling, tapi kalau dipakai
        // untuk upscaling hasilnya lebih blur/kurang detail,
        // yang justru mengurangi presisi landmark di jarak jauh).
        // =====================================================
        cv::Mat resized;
        int interp = (side < INPUT_SIZE) ? cv::INTER_CUBIC : cv::INTER_AREA;
        cv::resize(square, resized, cv::Size(INPUT_SIZE, INPUT_SIZE), 0, 0, interp);
        // =====================================================
        // 6. BGR → RGB
        // =====================================================
        cv::Mat rgb;
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
        // =====================================================
        // 7. FLOAT32 NORMALIZED 0..1
        //
        // Model input:
        // [1,224,224,3]
        // =====================================================
        std::vector<float> inputTensorValues;
        inputTensorValues.resize(INPUT_SIZE * INPUT_SIZE * 3);
        size_t index = 0;
        for (int y = 0; y < INPUT_SIZE; ++y) {
            for (int x = 0; x < INPUT_SIZE; ++x) {
                cv::Vec3b pixel = rgb.at<cv::Vec3b>(y, x);
                inputTensorValues[index++] = pixel[0] / 255.0f;
                inputTensorValues[index++] = pixel[1] / 255.0f;
                inputTensorValues[index++] = pixel[2] / 255.0f;
            }
        }
        // =====================================================
        // 8. TENSOR SHAPE
        // =====================================================
        std::array<int64_t, 4> inputShape = {1, 224, 224, 3 };
        size_t inputTensorSize = inputTensorValues.size();
        // =====================================================
        // 9. ONNX MEMORY INFO
        // =====================================================
        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(memoryInfo, inputTensorValues.data(), inputTensorSize, inputShape.data(), inputShape.size());
        // =====================================================
        // 10. INPUT / OUTPUT NAME
        // =====================================================
        const char* inputNamePtr = inputName.c_str();
        std::vector<const char*> outputNamePtrs;
        for (const auto& name : outputNames) {
            outputNamePtrs.push_back(name.c_str());
        }
        // =====================================================
        // 11. INFERENCE
        // =====================================================
        auto outputs = session.Run(Ort::RunOptions{nullptr}, &inputNamePtr, &inputTensor, 1, outputNamePtrs.data(), outputNamePtrs.size());
        if (outputs.size() < 4) {
            std::cerr << "Output handpose kurang dari 4!\n";
            return {};
        }
        // =====================================================
        // 12. OUTPUT LANDMARK
        //
        // Output 0:
        // [1,63]
        //
        // 21 landmark × 3
        // =====================================================
        float* landmarksData = outputs[0].GetTensorMutableData<float>();
        // =====================================================
        // 13. CONFIDENCE
        //
        // Output 1:
        // [1,1]
        // =====================================================
        float* confidenceData = outputs[1].GetTensorMutableData<float>();
        float confidence = confidenceData[0];
        if (confidence < HAND_CONF_THRESHOLD) {
            return {};
        }
        // =====================================================
        // 14. KONVERSI 21 LANDMARK
        // =====================================================
        std::vector<cv::Point3f> landmarks;
        landmarks.reserve(21);
        for (int i = 0; i < 21; ++i) {
            float nx = landmarksData[i * 3 + 0];
            float ny = landmarksData[i * 3 + 1];
            float nz = landmarksData[i * 3 + 2];
            // Model landmark berada relatif
            // terhadap center 224x224.
            float localX = (nx - INPUT_SIZE / 2.0f);
            float localY = (ny - INPUT_SIZE / 2.0f);
            // Scale kembali ke square
            float scale = static_cast<float>(side) / INPUT_SIZE;
            float px = localX * scale;
            float py = localY * scale;
            float pz = nz * scale;
            // Kembalikan dari square ke kanvas ROI (rotasi+geser)
            float rotFrameX = x1 + offsetX + side / 2.0f + px;
            float rotFrameY = y1 + offsetY + side / 2.0f + py;
            landmarks.emplace_back(rotFrameX, rotFrameY, pz);
        }

        // =====================================================
        // 13B. ROTASI BALIK KE FRAME ASLI
        //
        // landmarks di atas masih dalam koordinat kanvas ROI.
        // Rotasi balik pakai invRotMat supaya posisinya kembali
        // presisi menempel di telapak tangan pada frame asli,
        // walau tangan sedang miring/melambai.
        // =====================================================
        {
            std::vector<cv::Point2f> pts2d;
            pts2d.reserve(landmarks.size());
            for (const auto& lm : landmarks) {
                pts2d.emplace_back(lm.x, lm.y);
            }
            std::vector<cv::Point2f> ptsOrig;
            cv::transform(pts2d, ptsOrig, invRotMat);
            for (size_t i = 0; i < landmarks.size(); ++i) {
                landmarks[i].x = ptsOrig[i].x;
                landmarks[i].y = ptsOrig[i].y;
            }
        }
        // =====================================================
        // 15. BOUNDING BOX LANDMARK
        // =====================================================
        float minX = static_cast<float>(frame.cols);
        float minY = static_cast<float>(frame.rows);
        float maxX = 0.0f;
        float maxY = 0.0f;
        for (const auto& p : landmarks) {
            minX = std::min(minX, p.x);
            minY = std::min(minY, p.y);
            maxX = std::max(maxX, p.x);
            maxY = std::max(maxY, p.y);
        }
        minX = std::max(0.0f, std::min(minX, static_cast<float>(frame.cols - 1)));
        minY = std::max(0.0f, std::min(minY, static_cast<float>(frame.rows - 1)));
        maxX = std::max(0.0f, std::min(maxX, static_cast<float>(frame.cols - 1)));
        maxY = std::max(0.0f, std::min(maxY, static_cast<float>(frame.rows - 1)));
        cv::Rect2f bbox(minX, minY, std::max(1.0f, maxX - minX), std::max(1.0f, maxY - minY));
        return {landmarks, bbox, confidence};
    }
};

// ============================================================
// DRAW HAND CONNECTIONS
// ============================================================
void drawHand(cv::Mat& frame, const std::vector<cv::Point3f>& lm){
    if (lm.size() != 21)
        return;
    // --------------------------------------------------------
    // Hand skeleton
    // --------------------------------------------------------
    const std::vector<std::pair<int, int>> connections = {
        // Thumb
        {0,1},
        {1,2},
        {2,3},
        {3,4},

        // Index
        {0,5},
        {5,6},
        {6,7},
        {7,8},

        // Middle
        {0,9},
        {9,10},
        {10,11},
        {11,12},

        // Ring
        {0,13},
        {13,14},
        {14,15},
        {15,16},

        // Pinky
        {0,17},
        {17,18},
        {18,19},
        {19,20}
    };

    // --------------------------------------------------------
    // Lines
    // --------------------------------------------------------
    for (const auto& c : connections) {
        cv::Point p1(static_cast<int>(lm[c.first].x), static_cast<int>(lm[c.first].y));
        cv::Point p2(static_cast<int>(lm[c.second].x), static_cast<int>(lm[c.second].y));
        cv::line(frame, p1, p2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }
    // --------------------------------------------------------
    // Landmark points
    // --------------------------------------------------------
    for (int i = 0; i < 21; ++i) {
        cv::Point p(static_cast<int>(lm[i].x), static_cast<int>(lm[i].y));
        cv::circle(frame, p, 5, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    }
    // --------------------------------------------------------
    // Titik pusat telapak tangan (rata-rata landmark
    // wrist + 4 pangkal jari: 0, 5, 9, 13, 17)
    //
    // Ini titik yang dipakai WaveDetector sebagai acuan
    // gerakan melambai, digambar berbeda (kuning + crosshair)
    // supaya terlihat jelas bahwa titik acuannya memang tepat
    // menempel di telapak tangan, bukan di wrist saja.
    // --------------------------------------------------------
    cv::Point2f palmCenter(0.0f, 0.0f);
    const int palmIdx[5] = {0, 5, 9, 13, 17};
    for (int idx : palmIdx) {
        palmCenter.x += lm[idx].x;
        palmCenter.y += lm[idx].y;
    }
    palmCenter.x /= 5.0f;
    palmCenter.y /= 5.0f;
    cv::Point palmCenterPt(static_cast<int>(palmCenter.x), static_cast<int>(palmCenter.y));
    cv::circle(frame, palmCenterPt, 8, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::drawMarker(frame, palmCenterPt, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 14, 2, cv::LINE_AA);
}

// ============================================================
// WAVE DETECTOR
// ============================================================
class WaveDetector {
private:
    std::deque<float> history;
    static constexpr int HISTORY_SIZE = 25;
    // =========================================================
    // CEK APAKAH JARI TERENTANG (bukan fist/mengepal)
    //
    // Heuristik sederhana: untuk 4 jari (index, middle, ring,
    // pinky), jari dianggap "terentang" kalau jarak wrist->tip
    // lebih jauh daripada wrist->pip (buku jari tengah). Saat
    // fist/mengepal, tip melipat ke dalam sehingga jaraknya ke
    // wrist JADI LEBIH DEKAT daripada pip -> tidak dianggap
    // terentang.
    //
    // Margin 1.1x dipakai sebagai sedikit histeresis supaya
    // tidak flip-flop akibat noise landmark kecil.
    // =========================================================
    static bool isFingerExtended(const std::vector<cv::Point3f>& lm, int pipIdx, int tipIdx) {
        float wristToPip = std::hypot(lm[pipIdx].x - lm[0].x, lm[pipIdx].y - lm[0].y);
        float wristToTip = std::hypot(lm[tipIdx].x - lm[0].x, lm[tipIdx].y - lm[0].y);
        return wristToTip > wristToPip * 1.1f;
    }

    static bool isHandOpen(const std::vector<cv::Point3f>& lm) {
        int extendedCount = 0;
        // Index (pip=6, tip=8)
        if (isFingerExtended(lm, 6, 8))
            extendedCount++;
        // Middle (pip=10, tip=12)
        if (isFingerExtended(lm, 10, 12))
            extendedCount++;
        // Ring (pip=14, tip=16)
        if (isFingerExtended(lm, 14, 16))
            extendedCount++;
        // Pinky (pip=18, tip=20)
        if (isFingerExtended(lm, 18, 20))
            extendedCount++;
        // Minimal 3 dari 4 jari terentang supaya dianggap
        // telapak terbuka (toleransi 1 jari kurang sempurna
        // karena estimasi landmark tidak selalu presisi 100%)
        return extendedCount >= 3;
    }
public:
    bool update(const std::vector<cv::Point3f>& landmarks) {
        if (landmarks.size() != 21)
            return false;
        // =====================================================
        // SYARAT UTAMA: tangan harus dalam posisi TERBUKA.
        //
        // Tanpa cek ini, tangan yang mengepal (fist) lalu
        // digerakkan ke kiri-kanan (atau bahkan hanya jitter
        // kecil dari noise landmark) akan salah terbaca sebagai
        // gestur melambai, karena algoritma di bawah cuma lihat
        // osilasi horizontal titik tengah telapak tanpa peduli
        // bentuk tangannya.
        //
        // History di-reset supaya gerakan sebelum tangan
        // mengepal tidak "menyambung" secara salah begitu
        // tangan terbuka lagi di posisi yang berbeda.
        // =====================================================
        if (!isHandOpen(landmarks)) {
            history.clear();
            return false;
        }
        // =====================================================
        // CENTER TELAPAK
        // =====================================================
        float centerX = (landmarks[0].x + landmarks[5].x + landmarks[9].x + landmarks[13].x + landmarks[17].x) / 5.0f;
        // =====================================================
        // UKURAN TANGAN
        // =====================================================
        float palmWidth = std::hypot(landmarks[5].x - landmarks[17].x, landmarks[5].y - landmarks[17].y);
        if (palmWidth < 10.0f)
            return false;
        // =====================================================
        // NORMALISASI
        // =====================================================
        float normalizedX = centerX / palmWidth;
        history.push_back(normalizedX);
        while (history.size() > HISTORY_SIZE) {
            history.pop_front();
        }
        if (history.size() < 10)
            return false;
        // =====================================================
        // RANGE
        // =====================================================
        float minX = history[0];
        float maxX = history[0];
        for (float x : history) {
            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
        }
        float movement = maxX - minX;
        // =====================================================
        // PERUBAHAN ARAH
        // =====================================================
        int directionChanges = 0;
        int previousDirection = 0;
        for (size_t i = 1; i < history.size(); ++i) {
            float delta = history[i] - history[i - 1];
            int direction = 0;
            if (delta > 0.015f) {
                direction = 1;
            } else if (delta < -0.015f) {
                direction = -1;
            }
            if (direction != 0 && previousDirection != 0 && direction != previousDirection) {
                directionChanges++;
            }
            if (direction != 0) {
                previousDirection = direction;
            }
        }
        // =====================================================
        // WAVE
        // =====================================================
        return movement > 0.30f && directionChanges >= 2;
    }

    void reset() {
        history.clear();
    }
};

// ============================================================
// HAND LANDMARK TRACKER
// ============================================================
class HandTracker {
private:
    std::vector<cv::Point3f> previousLandmarks;
    std::vector<cv::Point3f> velocity;
    int lostFrames = 0;
    // Sebelumnya 8 frame -> landmark masih "diekstrapolasi"/
    // ditampilkan sampai ~8 frame setelah tangan benar-benar
    // hilang dari kamera, terlihat seperti landmark "hantu"
    // yang telat menghilang. Diturunkan ke 2 frame: cukup untuk
    // menoleransi 1 frame gagal deteksi sesaat (motion blur,
    // dsb) tanpa membuat tracking terlalu gampang reset, tapi
    // begitu tangan memang sudah tidak ada, landmark langsung
    // hilang dalam waktu yang nyaris tidak terasa.
    static constexpr int MAX_LOST_FRAMES = 2;
    bool tracking = false;
public:
    bool isTracking() const {
        return tracking;
    }

    void initialize(const std::vector<cv::Point3f>& landmarks) {
        if (landmarks.size() != 21)
            return;
        previousLandmarks = landmarks;
        velocity.resize(21, cv::Point3f(0, 0, 0));
        tracking = true;
        lostFrames = 0;
    }

    std::vector<cv::Point3f> update(const std::vector<cv::Point3f>& current) {
        if (current.size() != 21)
            return predict();
        if (!tracking) {
            initialize(current);
            return current;
        }
        constexpr float ALPHA = 0.75f;
        std::vector<cv::Point3f> result(21);
        for (int i = 0; i < 21; ++i) {
            float dx = current[i].x - previousLandmarks[i].x;
            float dy = current[i].y - previousLandmarks[i].y;
            float dz = current[i].z - previousLandmarks[i].z;
            // Smooth velocity
            velocity[i].x = 0.6f * velocity[i].x + 0.4f * dx;
            velocity[i].y = 0.6f * velocity[i].y + 0.4f * dy;
            velocity[i].z = 0.6f * velocity[i].z + 0.4f * dz;
            // Smooth landmark
            result[i].x = ALPHA * current[i].x + (1.0f - ALPHA) * previousLandmarks[i].x;
            result[i].y = ALPHA * current[i].y + (1.0f - ALPHA) * previousLandmarks[i].y;
            result[i].z = ALPHA * current[i].z + (1.0f - ALPHA) * previousLandmarks[i].z;
        }
        previousLandmarks = result;
        lostFrames = 0;
        return result;
    }

    std::vector<cv::Point3f> predict() {
        if (!tracking)
            return {};
        lostFrames++;
        if (lostFrames > MAX_LOST_FRAMES) {
            reset();
            return {};
        }
        std::vector<cv::Point3f> predicted = previousLandmarks;
        // Redam velocity supaya landmark prediksi tidak
        // "melesat" jauh mengikuti kecepatan gerak terakhir
        // selama masa toleransi singkat ini -- cenderung diam
        // di posisi terakhir, bukan terus melaju.
        constexpr float VELOCITY_DECAY = 0.5f;
        for (int i = 0; i < 21; ++i) {
            velocity[i].x *= VELOCITY_DECAY;
            velocity[i].y *= VELOCITY_DECAY;
            velocity[i].z *= VELOCITY_DECAY;
            predicted[i].x += velocity[i].x;
            predicted[i].y += velocity[i].y;
            predicted[i].z += velocity[i].z;
        }
        previousLandmarks = predicted;
        return predicted;
    }

    void reset() {
        tracking = false;
        lostFrames = 0;
        previousLandmarks.clear();
        velocity.clear();
    }
};

// ============================================================
// PERSON DETECTOR (YOLOv8n ONNX)
//
// Port dari vision.py: preprocess_onnx() + detect_persons().
// Beda dari palm/hand model di atas: YOLOv8 pakai layout input
// NCHW [1,3,H,W] (bukan NHWC), dan output-nya anchor-free
// [1, 4+numClasses, numBoxes] -- box didecode langsung tanpa
// anchor list terpisah.
// ============================================================
struct PersonDetection {
    cv::Rect box;
    float score;
};

class PersonDetector {
private:
    Ort::Session session{nullptr};
    Ort::SessionOptions sessionOptions;
    std::string inputName;
    std::string outputName;
    int inputWidth  = ONNX_INPUT_SIZE_FALLBACK;
    int inputHeight = ONNX_INPUT_SIZE_FALLBACK;
public:
    PersonDetector(Ort::Env& env, const std::string& modelPath) {
        std::cout << "Loading person (YOLOv8) model with ONNX Runtime...\n";
        // sessionOptions.SetIntraOpNumThreads(4);
        // sessionOptions.SetInterOpNumThreads(1);
        sessionOptions.SetInterOpNumThreads(2);
        sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session = Ort::Session(env, modelPath.c_str(), sessionOptions);
        Ort::AllocatorWithDefaultOptions allocator;
        {
            auto name = session.GetInputNameAllocated(0, allocator);
            inputName = name.get();
        }
        {
            auto name = session.GetOutputNameAllocated(0, allocator);
            outputName = name.get();
        }
        // Deteksi ukuran input dari model (mirip Python: cek
        // input_shape [1,3,H,W], fallback ke konstanta kalau
        // dynamic axes)
        auto inputShape = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (inputShape.size() == 4 && inputShape[2] > 0 && inputShape[3] > 0) {
            inputHeight = static_cast<int>(inputShape[2]);
            inputWidth  = static_cast<int>(inputShape[3]);
            std::cout << "YOLO input size terdeteksi dari model: " << inputWidth << "x" << inputHeight << "\n";
        } else {
            std::cout << "Model YOLO punya dynamic input shape, pakai " << "fallback " << inputWidth << "x" << inputHeight << "\n";
        }
        std::cout << "Person model loaded successfully (ONNX Runtime)\n";
    }

    std::vector<PersonDetection> detect(const cv::Mat& frame) {
        // ------------------------------------------------------
        // Letterbox (samakan persis dengan preprocess_onnx python,
        // termasuk warna padding abu-abu 114/114/114 khas YOLO)
        // ------------------------------------------------------
        int w0 = frame.cols;
        int h0 = frame.rows;
        float scale = std::min(static_cast<float>(inputWidth)  / w0, static_cast<float>(inputHeight) / h0);
        int newW = static_cast<int>(std::round(w0 * scale));
        int newH = static_cast<int>(std::round(h0 * scale));
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(newW, newH), 0, 0, cv::INTER_LINEAR);
        int padW = inputWidth  - newW;
        int padH = inputHeight - newH;
        int left = padW / 2;
        int top  = padH / 2;
        cv::Mat padded;
        cv::copyMakeBorder(resized, padded, top, padH - top, left, padW - left, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
        cv::Mat rgb;
        cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
        cv::Mat floatImg;
        rgb.convertTo(floatImg, CV_32FC3, 1.0 / 255.0);
        // HWC -> CHW (YOLOv8 butuh NCHW, beda dari palm/hand
        // model di atas yang NHWC)
        std::vector<cv::Mat> channels(3);
        cv::split(floatImg, channels);
        size_t planeSize = static_cast<size_t>(inputWidth) * inputHeight;
        std::vector<float> inputTensorValues(planeSize * 3);
        for (int c = 0; c < 3; ++c) {
            std::memcpy(inputTensorValues.data() + c * planeSize, channels[c].data, planeSize * sizeof(float));
        }
        std::array<int64_t, 4> inputShape = {1, 3, inputHeight, inputWidth};
        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(memoryInfo, inputTensorValues.data(), inputTensorValues.size(), inputShape.data(), inputShape.size());
        const char* inputNamePtr  = inputName.c_str();
        const char* outputNamePtr = outputName.c_str();
        auto outputs = session.Run(Ort::RunOptions{nullptr}, &inputNamePtr, &inputTensor, 1, &outputNamePtr, 1);
        if (outputs.empty()) {
            return {};
        }
        auto shapeOut = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
        // shape: [1, 4+numClasses, numBoxes]
        if (shapeOut.size() != 3) {
            return {};
        }
        int64_t numAttrs = shapeOut[1];
        int64_t numBoxes = shapeOut[2];
        int numClasses = static_cast<int>(numAttrs - 4);
        if (numClasses <= 0) {
            return {};
        }
        const float* data = outputs[0].GetTensorData<float>();
        // --------------------------------------------------
        // Decode: output YOLOv8 layout-nya (1, 4+numClasses,
        // numBoxes) -- artinya per-atribut kontigu (bukan
        // per-box), jadi index-nya data[attr*numBoxes + box].
        // --------------------------------------------------
        std::vector<cv::Rect> nmsBoxes;
        std::vector<float> confidences;
        for (int64_t i = 0; i < numBoxes; ++i) {
            float bestScore = -1.0f;
            int bestClass = -1;
            for (int c = 0; c < numClasses; ++c) {
                float s = data[(4 + c) * numBoxes + i];
                if (s > bestScore) {
                    bestScore = s;
                    bestClass = c;
                }
            }
            if (bestClass != ONNX_PERSON_CLASS_ID || bestScore < ONNX_CONF_THRESHOLD) {
                continue;
            }
            float cx = data[0 * numBoxes + i];
            float cy = data[1 * numBoxes + i];
            float w  = data[2 * numBoxes + i];
            float h  = data[3 * numBoxes + i];
            float x1 = (cx - w / 2.0f - left) / scale;
            float y1 = (cy - h / 2.0f - top)  / scale;
            float bw = w / scale;
            float bh = h / scale;
            nmsBoxes.emplace_back(static_cast<int>(x1), static_cast<int>(y1), static_cast<int>(bw), static_cast<int>(bh));
            confidences.push_back(bestScore);
        }
        if (nmsBoxes.empty()) {
            return {};
        }
        std::vector<int> keep;
        cv::dnn::NMSBoxes(nmsBoxes, confidences, ONNX_CONF_THRESHOLD, ONNX_NMS_THRESHOLD, keep);
        std::vector<PersonDetection> results;
        for (int idx : keep) {
            cv::Rect box = nmsBoxes[idx];
            box.x = std::max(0, box.x);
            box.y = std::max(0, box.y);
            box.width = std::min(box.width, frame.cols - box.x);
            box.height = std::min(box.height, frame.rows - box.y);
            if (box.width > 0 && box.height > 0) {
                results.push_back({box, confidences[idx]});
            }
        }
        return results;
    }
};

// ============================================================
// PERSON TRACK
//
// Port dari vision.py: create_track()/match_tracks(). Satu
// PersonTrack merepresentasikan satu orang yang dilacak lintas
// frame (dicocokkan berdasarkan jarak titik tengah bounding box).
//
// CATATAN PENTING (penyesuaian yang disengaja dari vision.py):
// Versi Python memakai MediaPipe Pose lebih dulu untuk
// mempersempit ROI ke area lengan (bahu-siku-pergelangan)
// sebelum menjalankan MediaPipe Hands di situ. Model Pose
// (BlazePose ONNX) BELUM diintegrasikan di C++ ini -- itu model
// terpisah yang butuh riset/verifikasi tersendiri seperti yang
// sudah dilakukan untuk model palm/hand di atas.
//
// Sebagai gantinya, palm+hand detector yang SUDAH ada dan sudah
// diperbaiki akurasinya (PalmDetector + HandPose + WaveDetector +
// HandTracker) langsung dijalankan pada ROI kotak orang tsb
// (dengan sedikit padding), tanpa penyempitan ke area lengan
// spesifik dulu. Tujuan akhirnya sama persis (deteksi tangan
// melambai per-orang), cuma area pencariannya sedikit lebih
// lebar dibanding versi Python yang sudah dipersempit ke lengan.
// Kalau butuh presisi/kecepatan setara Python, integrasi model
// Pose bisa ditambahkan menyusul.
// ============================================================
struct PersonTrack {
    int id = 0;
    cv::Rect box;
    cv::Point2f center;
    int handMiss  = 0;
    int trackMiss = 0;
    bool waving = false;
    cv::Rect handBox;
    bool hasHandLandmarks = false;
    std::vector<cv::Point3f> handLandmarks;
    // Tiap orang butuh state wave/tracker sendiri-sendiri,
    // tidak boleh dipakai bersama (shared) antar orang.
    WaveDetector waveDetector;
    HandTracker  handTracker;
};

class PersonTrackManager {
private:
    std::vector<PersonTrack> tracks;
    int nextTrackId = 1;
public:
    std::vector<PersonTrack>& getTracks() {
        return tracks;
    }
    void update(const std::vector<PersonDetection>& detections) {
        std::vector<cv::Rect> unmatched;
        for (const auto& d : detections) {
            unmatched.push_back(d.box);
        }
        std::vector<PersonTrack> result;
        for (auto& track : tracks) {
            int bestIdx = -1;
            float bestDist = TRACK_MATCH_MAX_DIST;
            for (size_t i = 0; i < unmatched.size(); ++i) {
                cv::Point2f c(unmatched[i].x + unmatched[i].width / 2.0f, unmatched[i].y + unmatched[i].height / 2.0f);
                float dist = static_cast<float>(cv::norm(c - track.center));
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = static_cast<int>(i);
                }
            }
            if (bestIdx >= 0) {
                track.box = unmatched[bestIdx];
                track.center = cv::Point2f(track.box.x + track.box.width / 2.0f, track.box.y + track.box.height / 2.0f);
                track.trackMiss = 0;
                unmatched.erase(unmatched.begin() + bestIdx);
                result.push_back(std::move(track));
            } else {
                track.trackMiss++;
                if (track.trackMiss <= TRACK_MAX_MISS) {
                    result.push_back(std::move(track));
                }
            }
        }
        for (const auto& box : unmatched) {
            PersonTrack t;
            t.id = nextTrackId++;
            t.box = box;
            t.center = cv::Point2f(box.x + box.width / 2.0f, box.y + box.height / 2.0f);
            result.push_back(std::move(t));
        }
        tracks = std::move(result);
    }
};

// ============================================================
// UPDATE STATUS MELAMBAI PER ORANG
//
// Port konseptual dari vision.py: update_track_wave_status().
// Lihat catatan di komentar PersonTrack di atas soal perbedaan
// (tanpa penyempitan ROI lewat model Pose).
// ============================================================
void updateTrackWaveStatus(PersonTrack& track, const cv::Mat& frame, PalmDetector& palmDetector, HandPose& handPose) {
    int padX = static_cast<int>(track.box.width * ARM_ROI_PADDING_RATIO);
    int padY = static_cast<int>(track.box.height * ARM_ROI_PADDING_RATIO);
    int x1 = std::max(0, track.box.x - padX);
    int y1 = std::max(0, track.box.y - padY);
    int x2 = std::min(frame.cols, track.box.x + track.box.width + padX);
    // int y2 = std::min(frame.rows, track.box.y + track.box.height + padY);
    int y2 = std::min(frame.rows, track.box.y + static_cast<int>(track.box.height * 0.6) + padY);
    if (x2 <= x1 || y2 <= y1) {
        track.hasHandLandmarks = false;
        return;
    }
    cv::Rect roiRect(x1, y1, x2 - x1, y2 - y1);
    cv::Mat personRoi = frame(roiRect);
    // std::vector<PalmDetection> palms = track.handTracker.isTracking() ? palmDetector.detect(personRoi) : palmDetector.detectMultiScale(personRoi);
    std::vector<PalmDetection> palms = palmDetector.detect(personRoi);
    bool newDetection = false;
    HandResult hand;
    if (!palms.empty()) {
        hand = handPose.estimate(personRoi, palms[0]);
        if (hand.landmarks.size() == 21) {
            // Geser koordinat landmark dari ruang ROI orang
            // ke ruang frame penuh
            for (auto& p : hand.landmarks) {
                p.x += static_cast<float>(roiRect.x);
                p.y += static_cast<float>(roiRect.y);
            }
            newDetection = true;
        }
    }
    std::vector<cv::Point3f> trackedLandmarks;
    if (newDetection) {
        trackedLandmarks = track.handTracker.update(hand.landmarks);
        track.handMiss = 0;
    } else if (track.handTracker.isTracking()) {
        trackedLandmarks = track.handTracker.predict();
    }
    if (trackedLandmarks.size() == 21) {
        track.hasHandLandmarks = true;
        track.handLandmarks = trackedLandmarks;
        float minX = static_cast<float>(frame.cols);
        float minY = static_cast<float>(frame.rows);
        float maxX = 0.0f;
        float maxY = 0.0f;
        for (const auto& p : trackedLandmarks) {
            minX = std::min(minX, p.x);
            minY = std::min(minY, p.y);
            maxX = std::max(maxX, p.x);
            maxY = std::max(maxY, p.y);
        }
        track.handBox = cv::Rect(static_cast<int>(minX), static_cast<int>(minY), static_cast<int>(std::max(1.0f, maxX - minX)), static_cast<int>(std::max(1.0f, maxY - minY)));
        track.waving = track.waveDetector.update(trackedLandmarks);
    } else {
        track.hasHandLandmarks = false;
        track.handMiss++;
        if (track.handMiss > WAVE_MISS_RESET_FRAMES) {
            track.waveDetector.reset();
            track.waving = false;
        }
    }
}

// ============================================================
// UI HELPER
// Port dari vision.py: put_text_with_bg()
// ============================================================
void putTextWithBg(cv::Mat& img, const std::string& text, cv::Point org, cv::Scalar color = cv::Scalar(255, 255, 255), cv::Scalar bgColor = cv::Scalar(0, 0, 0), double fontScale = 0.7, int thickness = 2, int padding = 6, double alpha = 0.55) {
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);
    cv::Mat overlay = img.clone();
    cv::rectangle(overlay, cv::Point(std::max(0, org.x - padding), std::max(0, org.y - textSize.height - padding)), cv::Point(std::min(img.cols, org.x + textSize.width + padding), std::min(img.rows, org.y + baseline + padding)), bgColor, -1);
    cv::addWeighted(overlay, alpha, img, 1.0 - alpha, 0, img);
    cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv::LINE_AA);
}

// =============================================================================
// KELAS BUFFER WEBRTC
// =============================================================================
class SharedFrameBuffer {
public:
  void update(const cv::Mat & frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_ = frame.clone();
  }
  bool get(cv::Mat & out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame_.empty()) {return false;}
    out = frame_.clone();
    return true;
  }
private:
  mutable std::mutex mutex_;
  cv::Mat frame_;
};

// =============================================================================
// KELAS SERVER WEBRTC (GStreamer)
// =============================================================================
class GstWebRtcServer {
public:
  GstWebRtcServer(SharedFrameBuffer & buf, uint16_t port, double disconnect_grace_sec)
  : frame_buffer_(buf), port_(port), disconnect_grace_sec_(disconnect_grace_sec) {}

  ~GstWebRtcServer() {stop();}

  void start() {
    running_ = true;
    daemon_ = MHD_start_daemon(
      MHD_USE_INTERNAL_POLLING_THREAD, port_, nullptr, nullptr,
      &GstWebRtcServer::onHttpRequest, this, MHD_OPTION_END);
    if (!daemon_) {
      throw std::runtime_error("Gagal start libmicrohttpd di port " + std::to_string(port_));
    }
    std::cout << "[webrtc] Signaling server 0.0.0.0:" << port_ << " (POST /webrtc/offer)\n";
  }

  void stop() {
    if (!running_.exchange(false)) {return;}
    std::lock_guard<std::mutex> lock(peers_mutex_);
    for (auto & [id, peer] : peers_) {
      peer->closing = true;
      if (peer->pipeline) {
        gst_element_set_state(peer->pipeline, GST_STATE_NULL);
        gst_object_unref(peer->pipeline);
      }
      if (peer->push_thread.joinable()) {peer->push_thread.join();}
    }
    peers_.clear();
    if (daemon_) {MHD_stop_daemon(daemon_); daemon_ = nullptr;}
  }

private:
  struct Peer {
    std::string id;
    GstElement * pipeline = nullptr;
    GstElement * appsrc = nullptr;
    GstElement * webrtcbin = nullptr;
    std::atomic<bool> closing{false};
    std::atomic<bool> ice_done{false};
    std::mutex ice_mutex;
    std::condition_variable ice_cv;
    std::thread push_thread;
  };

  static MHD_Result onHttpRequest(
    void * cls, struct MHD_Connection * connection, const char * url,
    const char * method, const char * /*version*/,
    const char * upload_data, size_t * upload_data_size, void ** con_cls) {
    auto * self = static_cast<GstWebRtcServer *>(cls);

    if (std::string(method) == "OPTIONS") {
      auto * resp = MHD_create_response_from_buffer(0, nullptr, MHD_RESPMEM_PERSISTENT);
      MHD_add_response_header(resp, "Access-Control-Allow-Origin", "*");
      MHD_add_response_header(resp, "Access-Control-Allow-Headers", "*");
      auto ret = MHD_queue_response(connection, MHD_HTTP_NO_CONTENT, resp);
      MHD_destroy_response(resp);
      return ret;
    }

    if (std::string(url) != "/webrtc/offer" || std::string(method) != "POST") {
      auto * resp = MHD_create_response_from_buffer(0, nullptr, MHD_RESPMEM_PERSISTENT);
      auto ret = MHD_queue_response(connection, MHD_HTTP_NOT_FOUND, resp);
      MHD_destroy_response(resp);
      return ret;
    }

    auto * body_accum = static_cast<std::string *>(*con_cls);
    if (body_accum == nullptr) {
      *con_cls = new std::string();
      return MHD_YES;
    }
    if (*upload_data_size != 0) {
      body_accum->append(upload_data, *upload_data_size);
      *upload_data_size = 0;
      return MHD_YES;
    }

    std::string response_body = self->handleOffer(*body_accum);
    delete body_accum;

    auto * resp = MHD_create_response_from_buffer(
      response_body.size(), const_cast<char *>(response_body.data()), MHD_RESPMEM_MUST_COPY);
    MHD_add_response_header(resp, "Content-Type", "application/json");
    MHD_add_response_header(resp, "Access-Control-Allow-Origin", "*");
    auto ret = MHD_queue_response(connection, MHD_HTTP_OK, resp);
    MHD_destroy_response(resp);
    return ret;
  }

  std::string handleOffer(const std::string & body) {
    json params;
    try {
      params = json::parse(body);
    } catch (...) {
      return json{{"status", "error"}, {"message", "Payload SDP tidak lengkap"}}.dump();
    }
    
    static std::atomic<int> peer_counter{0};
    auto peer = std::make_shared<Peer>();
    peer->id = "peer-" + std::to_string(peer_counter++);

    const char * encoders[] = {"vp8enc", "x264enc"};
    std::string encoder_bin;
    std::string encoding_name;

    for (const char * enc : encoders) {
      GstElementFactory * f = gst_element_factory_find(enc);
      if (f) {
        gst_object_unref(f);
        if (std::string(enc) == "vp8enc") {
          encoder_bin = "vp8enc deadline=1 error-resilient=1 cpu-used=8 threads=4 ! rtpvp8pay";
          encoding_name = "VP8";
        } else if (std::string(enc) == "x264enc") {
          encoder_bin = "x264enc speed-preset=ultrafast tune=zerolatency threads=4 key-int-max=15 ! h264parse config-interval=1 ! rtph264pay pt=96 name=pay0 config-interval=1";
          encoding_name = "H264";
        }
        break;
      }
    }

    std::ostringstream pipeline_desc;
    pipeline_desc
      << "appsrc name=src is-live=true block=true format=time do-timestamp=true "
      << "caps=video/x-raw,format=BGR,width=640,height=360,framerate=15/1 ! "
      << "videoconvert ! " << encoder_bin << " ! "
      << "application/x-rtp,media=video,encoding-name=" << encoding_name
      << " ! webrtcbin name=sendrecv bundle-policy=max-bundle";

    GError * err = nullptr;
    peer->pipeline = gst_parse_launch(pipeline_desc.str().c_str(), &err);
    peer->appsrc = gst_bin_get_by_name(GST_BIN(peer->pipeline), "src");
    peer->webrtcbin = gst_bin_get_by_name(GST_BIN(peer->pipeline), "sendrecv");

    g_signal_connect(peer->webrtcbin, "notify::ice-gathering-state",
      G_CALLBACK(+[](GstElement * webrtcbin, GParamSpec *, gpointer user_data) {
        auto * p = static_cast<Peer *>(user_data);
        GstWebRTCICEGatheringState state;
        g_object_get(webrtcbin, "ice-gathering-state", &state, nullptr);
        if (state == GST_WEBRTC_ICE_GATHERING_STATE_COMPLETE) {
          std::lock_guard<std::mutex> lk(p->ice_mutex);
          p->ice_done = true;
          p->ice_cv.notify_all();
        }
      }), peer.get());

    gst_element_set_state(peer->pipeline, GST_STATE_READY);

    GstSDPMessage * sdp_msg = nullptr;
    gst_sdp_message_new(&sdp_msg);
    std::string sdp_str = params["sdp"].get<std::string>();
    gst_sdp_message_parse_buffer(reinterpret_cast<const guint8 *>(sdp_str.c_str()), sdp_str.size(), sdp_msg);
    GstWebRTCSessionDescription * offer_desc = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp_msg);

    struct AnswerCtx { GstWebRtcServer * self; std::shared_ptr<Peer> peer; };
    auto * answer_ctx = new AnswerCtx{this, peer};

    GstPromise * set_remote_promise = gst_promise_new_with_change_func(
      +[](GstPromise *, gpointer data) {
        auto * ctx = static_cast<AnswerCtx *>(data);
        GstPromise * answer_promise = gst_promise_new_with_change_func(
          +[](GstPromise * promise, gpointer data2) {
            auto * ctx2 = static_cast<AnswerCtx *>(data2);
            const GstStructure * reply = gst_promise_get_reply(promise);
            GstWebRTCSessionDescription * answer = nullptr;
            gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, nullptr);
            gst_promise_unref(promise);
            GstPromise * local_promise = gst_promise_new();
            g_signal_emit_by_name(ctx2->peer->webrtcbin, "set-local-description", answer, local_promise);
            gst_promise_interrupt(local_promise);
            gst_promise_unref(local_promise);
            gst_webrtc_session_description_free(answer);
          }, ctx, nullptr);
        g_signal_emit_by_name(ctx->peer->webrtcbin, "create-answer", nullptr, answer_promise);
      }, answer_ctx, nullptr);
    g_signal_emit_by_name(peer->webrtcbin, "set-remote-description", offer_desc, set_remote_promise);
    gst_promise_interrupt(set_remote_promise);
    gst_promise_unref(set_remote_promise);
    gst_webrtc_session_description_free(offer_desc);

    gst_element_set_state(peer->pipeline, GST_STATE_PLAYING);

    {
      std::unique_lock<std::mutex> lk(peer->ice_mutex);
      peer->ice_cv.wait_for(lk, std::chrono::duration<double>(5.0), [&] {return peer->ice_done.load();});
    }

    GstWebRTCSessionDescription * local_desc = nullptr;
    g_object_get(peer->webrtcbin, "local-description", &local_desc, nullptr);
    gchar * sdp_text = gst_sdp_message_as_text(local_desc->sdp);
    std::string answer_sdp(sdp_text);
    g_free(sdp_text);
    gst_webrtc_session_description_free(local_desc);

    peer->push_thread = std::thread(&GstWebRtcServer::pushLoop, this, peer);

    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      peers_[peer->id] = peer;
    }

    return json{{"sdp", answer_sdp}, {"type", "answer"}}.dump();
  }

  void pushLoop(std::shared_ptr<Peer> peer) {
    constexpr int kW = 640, kH = 360, kFps = 15;
    auto interval = std::chrono::milliseconds(1000 / kFps);
    while (running_ && !peer->closing) {
      auto t0 = std::chrono::steady_clock::now();
      cv::Mat frame;
      if (!frame_buffer_.get(frame)) {
        frame = cv::Mat::zeros(kH, kW, CV_8UC3);
      }
      gsize size = frame.total() * frame.elemSize();
      GstBuffer * buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
      GstMapInfo map;
      gst_buffer_map(buffer, &map, GST_MAP_WRITE);
      std::memcpy(map.data, frame.data, size);
      gst_buffer_unmap(buffer, &map);

      GstFlowReturn ret;
      g_signal_emit_by_name(peer->appsrc, "push-buffer", buffer, &ret);
      gst_buffer_unref(buffer);
      if (ret != GST_FLOW_OK) {break;}

      auto elapsed = std::chrono::steady_clock::now() - t0;
      if (elapsed < interval) {std::this_thread::sleep_for(interval - elapsed);}
    }
    if (peer->pipeline) {gst_element_set_state(peer->pipeline, GST_STATE_NULL);}
    std::lock_guard<std::mutex> lock(peers_mutex_);
    peers_.erase(peer->id);
  }

  SharedFrameBuffer & frame_buffer_;
  uint16_t port_;
  double disconnect_grace_sec_;
  struct MHD_Daemon * daemon_ = nullptr;
  std::atomic<bool> running_{false};
  std::mutex peers_mutex_;
  std::map<std::string, std::shared_ptr<Peer>> peers_;
};

class FPSCounter {
private:
    int count = 0;
    double start;
    double current_fps = 0.0;

public:
    FPSCounter() {
        start = static_cast<double>(cv::getTickCount());
    }

    double update() {
        count++;
        double now = static_cast<double>(cv::getTickCount());
        double sec = (now - start) / cv::getTickFrequency();

        // Update FPS setiap 1 detik
        if (sec >= 1.0) {
            current_fps = count / sec;
            count = 0;
            start = now;
        }
        return current_fps;
    }
};

// ... (Pastikan semua header OpenCV & ONNX Runtime sudah di-include)

class HawkeyeNativeVision : public rclcpp::Node {
public:
    HawkeyeNativeVision() : rclcpp::Node("hawkeye_native_vision_node") {
        rclcpp::QoS qos_reliable(rclcpp::KeepLast(10));
        qos_reliable.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_reliable.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // pub_statustext_ = create_publisher<mavros_msgs::msg::StatusText>("/mavros/mavros/statustext/send", qos_reliable);
        pub_statustext_ = create_publisher<mavros_msgs::msg::StatusText>("/hawkeye/vision_statustext", qos_reliable);
        sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            // "/mavros/mavros/global_position/global", rclcpp::SensorDataQoS(),
            "/koordinat/gps", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                current_lat_ = msg->latitude;
                current_lon_ = msg->longitude;
            });

        std::string yolo_path = declare_parameter<std::string>("person_model", PERSON_MODEL);
        std::string palm_path = declare_parameter<std::string>("palm_model", PALM_MODEL);
        std::string hand_path = declare_parameter<std::string>("handpose_model", HANDPOSE_MODEL);

        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "HAWKEYE_NATIVE");
        personDetector_ = std::make_unique<PersonDetector>(*env_, yolo_path);
        palmDetector_ = std::make_unique<PalmDetector>(*env_, palm_path);
        handPose_ = std::make_unique<HandPose>(*env_, hand_path);

        // Kamera dioptimalkan di resolusi 360p untuk RPi 5
        camera_ = std::make_unique<CameraReconnect>(0, 640, 360, 15, 0.5, 10.0, std::vector<int>{0, 1, 2});
        
        // INISIALISASI WEBRTC (Berjalan di port 5000)
        webrtc_server_ = std::make_unique<GstWebRtcServer>(frame_buffer_, 5000, 8.0);
    }

    ~HawkeyeNativeVision() {
        if (webrtc_server_) webrtc_server_->stop();
        if (camera_) camera_->release();
    }

    void run() {
        // Nyalakan server WebRTC
        webrtc_server_->start();
        RCLCPP_INFO(get_logger(), "WebRTC Server berjalan di port 5000");

        cv::Mat frame;
        long long frameCount = 0;
        int consecutiveErrors = 0;
        constexpr int MAX_CONSECUTIVE_FRAME_ERRORS = 30;
        constexpr int DETECTION_INTERVAL = 5; 

        // Gunakan FPSCounter
        FPSCounter fps_counter;
        double current_fps = 0.0;

        while (rclcpp::ok()) {
            rclcpp::spin_some(get_node_base_interface());

            if (!camera_->read(frame) || frame.empty()) {
                if (camera_->needsReconnect()) {
                    camera_->tryReconnect();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(30)); 
                continue;
            }

            try {
                frameCount++;
                // Update dan ambil nilai FPS terbaru
                current_fps = fps_counter.update();

                // Gandakan frame untuk digambar dan dikirim ke WebRTC
                cv::Mat displayFrame = frame.clone();
                bool shouldRunDetection = (frameCount % DETECTION_INTERVAL == 0);
                int jumlahOrang = static_cast<int>(trackManager_.getTracks().size());
                bool anyWaving = false;

                if (shouldRunDetection) {
                    std::vector<PersonDetection> rawBoxes = personDetector_->detect(frame);
                    trackManager_.update(rawBoxes);
                    
                    // for (auto& track : trackManager_.getTracks()) {
                    //     updateTrackWaveStatus(track, frame, *palmDetector_, *handPose_);
                    // }
                    // Batasi agar MediaPipe hanya mengevaluasi maksimal 2 orang
                    int max_processed = 0;
                    for (auto& track : trackManager_.getTracks()) {
                        if (max_processed >= 2) break;
                        updateTrackWaveStatus(track, frame, *palmDetector_, *handPose_);
                        max_processed++;
                    }
                    
                    jumlahOrang = static_cast<int>(rawBoxes.size());
                }

                // Cek status melambai
                for (const auto& track : trackManager_.getTracks()) {
                    if (track.waving) {
                        anyWaving = true;
                        break;
                    }
                }

                // Report MAVROS jika ada deteksi
                if (shouldRunDetection && jumlahOrang > 0) {
                    reportVictim(jumlahOrang, anyWaving);
                }

                // ==========================================================
                // GAMBAR OVERLAY (DIKIRIM KE WEBRTC, TANPA TAMPIL LOKAL)
                // ==========================================================
                for (auto& track : trackManager_.getTracks()) {
                    cv::rectangle(displayFrame, track.box, cv::Scalar(0, 255, 0), 2);
                    std::string label = "ID " + std::to_string(track.id) + (track.waving ? " - MELAMBAI" : "");
                    putTextWithBg(displayFrame, label, cv::Point(track.box.x, std::max(18, track.box.y - 10)), track.waving ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), track.waving ? cv::Scalar(0, 0, 60) : cv::Scalar(0, 60, 0));
                    
                    // if (track.hasHandLandmarks) {
                    //     drawHand(displayFrame, track.handLandmarks);
                    // }
                }
                
                putTextWithBg(displayFrame, jumlahOrang > 0 ? "KORBAN TERDETEKSI" : "MENCARI", cv::Point(20, 30), jumlahOrang > 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
                putTextWithBg(displayFrame, "JUMLAH: " + std::to_string(jumlahOrang), cv::Point(20, 60), cv::Scalar(0, 255, 255));
                putTextWithBg(displayFrame, "STATUS: " + std::string(anyWaving ? "MINTA TOLONG" : "NO STATUS"), cv::Point(20, 90), anyWaving ? cv::Scalar(0, 0, 255) : cv::Scalar(200, 200, 200));
                putTextWithBg(displayFrame, cv::format("FPS: %.2f", current_fps), cv::Point(20, 120), cv::Scalar(255, 255, 0));

                // cv::resize(displayFrame, displayFrame, cv::Size(640, 360));
                // UPDATE KE BUFFER WEBRTC
                frame_buffer_.update(displayFrame);
                
                consecutiveErrors = 0;

            } catch (const std::exception& exc) {
                consecutiveErrors++;
                if (consecutiveErrors >= MAX_CONSECUTIVE_FRAME_ERRORS) break;
            }
        }
    }

private:

    rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr pub_statustext_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    double current_lat_ = 0.0, current_lon_ = 0.0;
    std::chrono::steady_clock::time_point last_sent_time_ = std::chrono::steady_clock::now() - std::chrono::hours(1);

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<PersonDetector> personDetector_;
    std::unique_ptr<PalmDetector> palmDetector_;
    std::unique_ptr<HandPose> handPose_;
    std::unique_ptr<CameraReconnect> camera_;
    PersonTrackManager trackManager_;

    // Variabel WebRTC
    SharedFrameBuffer frame_buffer_;
    std::unique_ptr<GstWebRtcServer> webrtc_server_;

    void reportVictim(int jumlahOrang, bool jikaMelambai) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_sent_time_).count();
        if (elapsed <= 2.0) {
            return;
        }
        mavros_msgs::msg::StatusText msg;
        char buf[64];
        if (jikaMelambai) {
            msg.severity = 0; // EMERGENCY
            std::snprintf(buf, sizeof(buf), "[SAR]; H;%d;%.6f;%.6f", jumlahOrang, current_lat_, current_lon_);
        } else {
            msg.severity = 6; // NOTICE
            std::snprintf(buf, sizeof(buf), "[SAR]; D;%d;%.6f;%.6f", jumlahOrang, current_lat_, current_lon_);
        }
        std::string text(buf);
        if (text.size() > 50) {
            text.resize(50);
        }
        msg.text = text;
        pub_statustext_->publish(msg);
        RCLCPP_INFO(get_logger(), "[REPORT] StatusText -> severity=%d | text='%s'", msg.severity, msg.text.c_str());
        last_sent_time_ = now;
    }
};

// ============================================================
// MAIN
// ============================================================
int main(int argc, char ** argv) {
    // Inisialisasi Gstreamer untuk WebRTC WAJIB dilakukan paling awal
    gst_init(&argc, &argv); 
    
    // Inisialisasi ROS 2
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<HawkeyeNativeVision>();
        node->run(); 
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("hawkeye_native_vision_node"), "Fatal Error: %s", e.what());
    
    }
    rclcpp::shutdown();
    return 0;
}