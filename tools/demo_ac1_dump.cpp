// demo_ac1_dump: capture from a RoboSense AC1 over USB with rs_driver (dev_opt_AC1) and dump
// point clouds (PLY), images (raw NV12/RGB) and IMU samples (CSV) to an output directory.
// Based on rs_driver/demo/demo_usb.cpp (BSD-3, RoboSense).
#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#ifdef _WIN32
#include <direct.h>
#define MKDIR(p) _mkdir(p)
#else
#include <sys/stat.h>
#define MKDIR(p) mkdir(p, 0755)
#endif

typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloudMsg;
using namespace robosense::lidar;

static SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue, stuffed_cloud_queue;
static SyncQueue<std::shared_ptr<ImuData>> free_imu_queue, stuffed_imu_queue;
static SyncQueue<std::shared_ptr<ImageData>> free_image_queue, stuffed_image_queue;

static std::string g_out = "captures";
static int g_save_clouds = 3;
static int g_save_images = 3;
static std::atomic<uint32_t> n_cloud{0}, n_image{0}, n_imu{0}, n_err{0};
static std::atomic<bool> to_exit{false};

static bool validPoint(const PointT& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && (p.x != 0.f || p.y != 0.f || p.z != 0.f);
}

static std::shared_ptr<PointCloudMsg> getCloud() {
  auto m = free_cloud_queue.pop();
  return m ? m : std::make_shared<PointCloudMsg>();
}
static void putCloud(std::shared_ptr<PointCloudMsg> m) { stuffed_cloud_queue.push(m); }
static std::shared_ptr<ImuData> getImu() {
  auto m = free_imu_queue.pop();
  return m ? m : std::make_shared<ImuData>();
}
static void putImu(const std::shared_ptr<ImuData>& m) { stuffed_imu_queue.push(m); }
static std::shared_ptr<ImageData> getImage() {
  auto m = free_image_queue.pop();
  return m ? m : std::make_shared<ImageData>();
}
static void putImage(const std::shared_ptr<ImageData>& m) { stuffed_image_queue.push(m); }

static void onException(const Error& code) {
  n_err++;
  RS_WARNING << "[driver exception] " << code.toString() << RS_REND;
}

static std::string outPath(const std::string& name) { return g_out + "/" + name; }

static void processCloud() {
  while (!to_exit) {
    auto msg = stuffed_cloud_queue.popWait();
    if (!msg) continue;
    uint32_t idx = ++n_cloud;
    size_t valid = 0;
    float rmin = 1e9f, rmax = 0.f;
    for (const auto& p : msg->points) {
      if (validPoint(p)) {
        valid++;
        float r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (r < rmin) rmin = r;
        if (r > rmax) rmax = r;
      }
    }
    RS_MSG << "cloud #" << idx << " points=" << msg->points.size() << " valid=" << valid
           << " range[m]=" << std::fixed << std::setprecision(2) << (valid ? rmin : 0.f) << ".." << (valid ? rmax : 0.f)
           << " ts=" << std::setprecision(6) << msg->timestamp << " seq=" << msg->seq << RS_REND;
    if ((int)idx <= g_save_clouds) {
      std::ostringstream fn;
      fn << "cloud_" << std::setw(3) << std::setfill('0') << idx << ".ply";
      std::ofstream f(outPath(fn.str()));
      f << "ply\nformat ascii 1.0\ncomment RoboSense AC1 via rs_driver dev_opt_AC1, frame " << idx
        << " ts " << std::setprecision(6) << std::fixed << msg->timestamp << "\n"
        << "element vertex " << valid << "\nproperty float x\nproperty float y\nproperty float z\n"
        << "property uchar intensity\nproperty ushort ring\nend_header\n";
      for (const auto& p : msg->points) {
        if (validPoint(p))
          f << std::setprecision(4) << p.x << " " << p.y << " " << p.z << " " << (int)p.intensity << " " << p.ring << "\n";
      }
      RS_MSG << "  saved " << fn.str() << " (" << valid << " pts)" << RS_REND;
    }
    free_cloud_queue.push(msg);
  }
}

static const char* fmtName(frame_format_t f) {
  switch (f) {
    case FRAME_FORMAT_NV12: return "nv12";
    case FRAME_FORMAT_BGR24: return "bgr24";
    case FRAME_FORMAT_RGB24: return "rgb24";
    case FRAME_FORMAT_YUV422: return "yuyv";
    default: return "any";
  }
}

static void processImage() {
  while (!to_exit) {
    auto msg = stuffed_image_queue.popWait();
    if (!msg) continue;
    uint32_t idx = ++n_image;
    // NOTE: RS_MSG expands to two statements, so it must be braced under an if.
    if (idx == 1 || idx % 10 == 0) {
      RS_MSG << "image #" << idx << " " << msg->width << "x" << msg->height << " fmt=" << fmtName(msg->frame_format)
             << " bytes=" << msg->data_bytes << " ts=" << std::fixed << std::setprecision(6) << msg->timestamp << RS_REND;
    }
    if ((int)idx <= g_save_images && msg->data && msg->data_bytes > 0) {
      std::ostringstream fn;
      fn << "image_" << std::setw(3) << std::setfill('0') << idx << "_" << msg->width << "x" << msg->height << "_"
         << fmtName(msg->frame_format) << ".raw";
      std::ofstream f(outPath(fn.str()), std::ios::binary);
      f.write(reinterpret_cast<const char*>(msg->data.get()), msg->data_bytes);
      RS_MSG << "  saved " << fn.str() << RS_REND;
    }
    free_image_queue.push(msg);
  }
}

static void processImu() {
  std::ofstream csv(outPath("imu.csv"));
  csv << "timestamp,ax,ay,az,gx,gy,gz,qx,qy,qz,qw\n";
  while (!to_exit) {
    auto msg = stuffed_imu_queue.popWait();
    if (!msg) continue;
    uint32_t idx = ++n_imu;
    csv << std::fixed << std::setprecision(6) << msg->timestamp << "," << msg->linear_acceleration_x << ","
        << msg->linear_acceleration_y << "," << msg->linear_acceleration_z << "," << msg->angular_velocity_x << ","
        << msg->angular_velocity_y << "," << msg->angular_velocity_z << "," << msg->orientation_x << ","
        << msg->orientation_y << "," << msg->orientation_z << "," << msg->orientation_w << "\n";
    if (idx == 1 || idx % 100 == 0) {
      RS_MSG << "imu #" << idx << " acc=(" << std::fixed << std::setprecision(3) << msg->linear_acceleration_x << ","
             << msg->linear_acceleration_y << "," << msg->linear_acceleration_z << ") gyro=(" << msg->angular_velocity_x
             << "," << msg->angular_velocity_y << "," << msg->angular_velocity_z << ") ts=" << std::setprecision(6)
             << msg->timestamp << RS_REND;
    }
    free_imu_queue.push(msg);
  }
}

static void usage(const char* exe) {
  std::fprintf(stderr,
               "usage: %s [--seconds N] [--out DIR] [--usb200] [--no-image] [--width W] [--height H] [--fps F]\n"
               "          [--imu-fps F] [--save-clouds N] [--save-images N]\n",
               exe);
}

int main(int argc, char* argv[]) {
  int seconds = 10;
  bool usb200 = false, image = true;
  int width = 1920, height = 1080, fps = 30, imu_fps = 100;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    bool has_next = (i + 1 < argc);
    if (a == "--seconds" && has_next) seconds = std::atoi(argv[++i]);
    else if (a == "--out" && has_next) g_out = argv[++i];
    else if (a == "--usb200") usb200 = true;
    else if (a == "--no-image") image = false;
    else if (a == "--width" && has_next) width = std::atoi(argv[++i]);
    else if (a == "--height" && has_next) height = std::atoi(argv[++i]);
    else if (a == "--fps" && has_next) fps = std::atoi(argv[++i]);
    else if (a == "--imu-fps" && has_next) imu_fps = std::atoi(argv[++i]);
    else if (a == "--save-clouds" && has_next) g_save_clouds = std::atoi(argv[++i]);
    else if (a == "--save-images" && has_next) g_save_images = std::atoi(argv[++i]);
    else { usage(argv[0]); return 1; }
  }
  MKDIR(g_out.c_str());

  RS_TITLE << "rs_driver v" << getDriverVersion() << "  AC1 dump -> " << g_out << RS_REND;
  RSDriverParam param;
  param.input_type = InputType::USB;
  param.lidar_type = LidarType::RS_AC1;
  param.input_param.enable_usb200 = usb200;
  param.input_param.enable_image = image;
  param.input_param.image_width = width;
  param.input_param.image_height = height;
  param.input_param.image_fps = fps;
  param.input_param.imu_fps = imu_fps;
  param.print();

  LidarDriver<PointCloudMsg> driver;
  driver.regPointCloudCallback(getCloud, putCloud);
  driver.regExceptionCallback(onException);
  driver.regImuDataCallback(getImu, putImu);
  driver.regImageDataCallback(getImage, putImage);
  if (!driver.init(param)) {
    RS_ERROR << "driver.init failed" << RS_REND;
    return 2;
  }
  std::thread tc(processCloud), ti(processImage), tm(processImu);
  if (!driver.start()) {
    RS_ERROR << "driver.start failed" << RS_REND;
    to_exit = true;
    tc.join(); ti.join(); tm.join();
    return 3;
  }
  RS_MSG << "streaming for " << seconds << " s ..." << RS_REND;
  auto t0 = std::chrono::steady_clock::now();
  std::this_thread::sleep_for(std::chrono::seconds(seconds));
  driver.stop();
  to_exit = true;
  tc.join(); ti.join(); tm.join();
  double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  RS_TITLE << "SUMMARY: " << std::fixed << std::setprecision(1) << dt << " s  clouds=" << n_cloud << " (" << n_cloud / dt
           << "/s)  images=" << n_image << " (" << n_image / dt << "/s)  imu=" << n_imu << " (" << n_imu / dt
           << "/s)  driver_exceptions=" << n_err << RS_REND;
  return n_cloud > 0 ? 0 : 4;
}
