// demo_ac1_stream: stream RoboSense AC1 frames (point cloud, image, IMU) from rs_driver (dev_opt_AC1)
// over a localhost TCP socket, for tools/live_view.py, rerun_view.py, overlay_view.py, freeze_frame.py.
//
// Wire format (little-endian, packed):
//   header : char magic[4]="AC1F", uint8 type, uint8 pad[3], uint32 payload_len, double timestamp   (20 bytes)
//   type 1 cloud : uint32 n, uint32 grid_w, uint32 grid_h, then n * {float x, y, z, intensity}
//   type 2 image : uint32 w, uint32 h, uint32 fmt, uint32 nbytes, then nbytes raw pixels
//   type 3 imu   : float ax, ay, az, gx, gy, gz, qx, qy, qz, qw
// Any number of clients may connect; each gets the latest cloud/image and every IMU sample since it joined.
// Frames produced while nobody is connected are dropped. By default the process exits when the last client
// leaves (--keep-running to stay up).
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sock_t;
#define SOCK_INVALID INVALID_SOCKET
#define sock_close closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int sock_t;
#define SOCK_INVALID (-1)
#define sock_close close
#endif

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloudMsg;
using namespace robosense::lidar;

#pragma pack(push, 1)
struct FrameHeader {
  char magic[4];
  uint8_t type;
  uint8_t pad[3];
  uint32_t payload_len;
  double timestamp;
};
#pragma pack(pop)
static_assert(sizeof(FrameHeader) == 20, "FrameHeader must be 20 bytes");

static const uint32_t GRID_W = 96, GRID_H = 288;  // decoder_RSAC1: 288 lines x 96 points
static const size_t IMU_RING = 400;

static SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
static SyncQueue<std::shared_ptr<ImuData>> free_imu_queue;
static SyncQueue<std::shared_ptr<ImageData>> free_image_queue;

// latest-frame slots with sequence numbers; every client thread tracks what it has sent
static std::mutex g_mtx;
static std::condition_variable g_cv;
static std::vector<uint8_t> g_cloud_buf, g_image_buf;
static uint64_t g_cloud_seq = 0, g_image_seq = 0, g_imu_seq = 0;
static std::deque<std::pair<uint64_t, std::vector<uint8_t>>> g_imu_ring;
static std::atomic<int> g_clients{0};
static std::atomic<bool> g_had_client{false};
static std::atomic<bool> g_stop{false};
static std::atomic<uint32_t> n_cloud{0}, n_image{0}, n_imu{0}, n_err{0}, n_sent{0};

static void putHeader(std::vector<uint8_t>& b, uint8_t type, uint32_t payload_len, double ts) {
  FrameHeader h;
  std::memcpy(h.magic, "AC1F", 4);
  h.type = type;
  h.pad[0] = h.pad[1] = h.pad[2] = 0;
  h.payload_len = payload_len;
  h.timestamp = ts;
  b.resize(sizeof(h) + payload_len);
  std::memcpy(b.data(), &h, sizeof(h));
}
template <typename T>
static void put(std::vector<uint8_t>& b, size_t& off, const T& v) {
  std::memcpy(b.data() + off, &v, sizeof(T));
  off += sizeof(T);
}

static std::shared_ptr<PointCloudMsg> getCloud() {
  auto m = free_cloud_queue.pop();
  return m ? m : std::make_shared<PointCloudMsg>();
}
static void putCloud(std::shared_ptr<PointCloudMsg> msg) {
  n_cloud++;
  if (g_clients > 0) {
    std::vector<uint8_t> b;
    uint32_t n = (uint32_t)msg->points.size();
    putHeader(b, 1, 12 + n * 16, msg->timestamp);
    size_t off = sizeof(FrameHeader);
    put(b, off, n);
    put(b, off, GRID_W);
    put(b, off, GRID_H);
    for (const auto& p : msg->points) {
      put(b, off, p.x);
      put(b, off, p.y);
      put(b, off, p.z);
      put(b, off, (float)p.intensity);
    }
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      g_cloud_buf.swap(b);
      g_cloud_seq++;
    }
    g_cv.notify_all();
  }
  free_cloud_queue.push(msg);
}

static std::shared_ptr<ImageData> getImage() {
  auto m = free_image_queue.pop();
  return m ? m : std::make_shared<ImageData>();
}
static void putImage(const std::shared_ptr<ImageData>& msg) {
  n_image++;
  if (g_clients > 0 && msg->data && msg->data_bytes > 0) {
    std::vector<uint8_t> b;
    putHeader(b, 2, 16 + msg->data_bytes, msg->timestamp);
    size_t off = sizeof(FrameHeader);
    put(b, off, msg->width);
    put(b, off, msg->height);
    put(b, off, (uint32_t)msg->frame_format);
    put(b, off, msg->data_bytes);
    std::memcpy(b.data() + off, msg->data.get(), msg->data_bytes);
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      g_image_buf.swap(b);
      g_image_seq++;
    }
    g_cv.notify_all();
  }
  free_image_queue.push(msg);
}

static std::shared_ptr<ImuData> getImu() {
  auto m = free_imu_queue.pop();
  return m ? m : std::make_shared<ImuData>();
}
static void putImu(const std::shared_ptr<ImuData>& msg) {
  n_imu++;
  if (g_clients > 0) {
    std::vector<uint8_t> b;
    putHeader(b, 3, 40, msg->timestamp);
    size_t off = sizeof(FrameHeader);
    const float v[10] = {msg->linear_acceleration_x, msg->linear_acceleration_y, msg->linear_acceleration_z,
                         msg->angular_velocity_x,    msg->angular_velocity_y,    msg->angular_velocity_z,
                         msg->orientation_x,         msg->orientation_y,         msg->orientation_z,
                         msg->orientation_w};
    for (float f : v) put(b, off, f);
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      g_imu_ring.emplace_back(++g_imu_seq, std::move(b));
      while (g_imu_ring.size() > IMU_RING) g_imu_ring.pop_front();
    }
    g_cv.notify_all();
  }
  free_imu_queue.push(msg);
}

static void onException(const Error& code) {
  n_err++;
  RS_WARNING << "[driver exception] " << code.toString() << RS_REND;
}

static bool sendAll(sock_t s, const uint8_t* p, size_t n) {
  while (n > 0) {
    int k = send(s, (const char*)p, (int)n, 0);
    if (k <= 0) return false;
    p += k;
    n -= (size_t)k;
  }
  return true;
}

static void clientThread(sock_t cs, int id, bool exit_on_disconnect) {
  uint64_t last_c, last_i, last_m;
  {
    std::lock_guard<std::mutex> lk(g_mtx);
    last_c = g_cloud_seq;
    last_i = g_image_seq;
    last_m = g_imu_seq;
  }
  bool ok = true;
  while (ok && !g_stop) {
    std::vector<uint8_t> cloud, image, imu;
    {
      std::unique_lock<std::mutex> lk(g_mtx);
      g_cv.wait_for(lk, std::chrono::milliseconds(200),
                    [&] { return g_cloud_seq != last_c || g_image_seq != last_i || g_imu_seq != last_m || g_stop; });
      if (g_cloud_seq != last_c) { cloud = g_cloud_buf; last_c = g_cloud_seq; }
      if (g_image_seq != last_i) { image = g_image_buf; last_i = g_image_seq; }
      if (g_imu_seq != last_m) {
        for (const auto& e : g_imu_ring)
          if (e.first > last_m) imu.insert(imu.end(), e.second.begin(), e.second.end());
        last_m = g_imu_seq;
      }
    }
    if (!imu.empty()) ok = sendAll(cs, imu.data(), imu.size());
    if (ok && !cloud.empty()) { ok = sendAll(cs, cloud.data(), cloud.size()); if (ok) n_sent++; }
    if (ok && !image.empty()) ok = sendAll(cs, image.data(), image.size());
  }
  sock_close(cs);
  int left = --g_clients;
  RS_MSG << "viewer #" << id << " disconnected, " << left << " left" << RS_REND;
  if (left == 0 && exit_on_disconnect) g_stop = true;
}

static void acceptThread(int port, bool exit_on_disconnect) {
  sock_t ls = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, (const char*)&one, sizeof(one));
  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons((unsigned short)port);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(ls, (sockaddr*)&addr, sizeof(addr)) != 0 || listen(ls, 4) != 0) {
    RS_ERROR << "bind/listen on 127.0.0.1:" << port << " failed" << RS_REND;
    g_stop = true;
    return;
  }
  RS_MSG << "listening on 127.0.0.1:" << port << RS_REND;
  int next_id = 1;
  while (!g_stop) {
    sock_t cs = accept(ls, nullptr, nullptr);
    if (cs == SOCK_INVALID) break;
    setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
    int id = next_id++;
    int now = ++g_clients;
    g_had_client = true;
    RS_MSG << "viewer #" << id << " connected (" << now << " connected)" << RS_REND;
    std::thread(clientThread, cs, id, exit_on_disconnect).detach();
  }
  sock_close(ls);
}

static void usage(const char* exe) {
  std::fprintf(stderr,
               "usage: %s [--port P] [--uuid SERIAL] [--usb200] [--no-image] [--width W] [--height H] [--fps F] [--imu-fps F]\n"
               "          [--seconds N] [--keep-running]   (default: exit when the last viewer disconnects)\n"
               "  two units: run one streamer per unit, e.g. --uuid 1111bfaf0060 --port 5577 and --uuid 1111bfa90008 --port 5578\n",
               exe);
}

int main(int argc, char* argv[]) {
  int port = 5577, seconds = 0, width = 640, height = 480, fps = 30, imu_fps = 100;
  bool usb200 = false, image = true, exit_on_disconnect = true;
  std::string uuid;  // device serial, e.g. 1111bfaf0060; empty = first AC1 found
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    bool has_next = (i + 1 < argc);
    if (a == "--port" && has_next) port = std::atoi(argv[++i]);
    else if (a == "--uuid" && has_next) uuid = argv[++i];
    else if (a == "--seconds" && has_next) seconds = std::atoi(argv[++i]);
    else if (a == "--usb200") usb200 = true;
    else if (a == "--no-image") image = false;
    else if (a == "--width" && has_next) width = std::atoi(argv[++i]);
    else if (a == "--height" && has_next) height = std::atoi(argv[++i]);
    else if (a == "--fps" && has_next) fps = std::atoi(argv[++i]);
    else if (a == "--imu-fps" && has_next) imu_fps = std::atoi(argv[++i]);
    else if (a == "--keep-running") exit_on_disconnect = false;
    else { usage(argv[0]); return 1; }
  }
#ifdef _WIN32
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { std::fprintf(stderr, "WSAStartup failed\n"); return 1; }
#endif

  RS_TITLE << "rs_driver v" << getDriverVersion() << "  AC1 stream on 127.0.0.1:" << port << RS_REND;
  RSDriverParam param;
  param.input_type = InputType::USB;
  param.lidar_type = LidarType::RS_AC1;
  param.input_param.enable_usb200 = usb200;
  param.input_param.enable_image = image;
  param.input_param.image_width = width;
  param.input_param.image_height = height;
  param.input_param.image_fps = fps;
  param.input_param.imu_fps = imu_fps;
  param.input_param.device_uuid = uuid;

  LidarDriver<PointCloudMsg> driver;
  driver.regPointCloudCallback(getCloud, putCloud);
  driver.regExceptionCallback(onException);
  driver.regImuDataCallback(getImu, putImu);
  driver.regImageDataCallback(getImage, putImage);
  if (!driver.init(param)) { RS_ERROR << "driver.init failed" << RS_REND; return 2; }
  if (!driver.start()) { RS_ERROR << "driver.start failed" << RS_REND; return 3; }
  RS_MSG << "streaming " << width << "x" << height << "@" << fps << " image, 10 Hz cloud, " << imu_fps << " Hz imu"
         << (usb200 ? " (usb 2.0 mode)" : "") << RS_REND;

  std::thread net(acceptThread, port, exit_on_disconnect);
  auto t0 = std::chrono::steady_clock::now();
  uint32_t last_cloud = 0;
  while (!g_stop) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (seconds > 0 && dt >= seconds) g_stop = true;
    uint32_t c = n_cloud;
    if (((int)dt % 5) == 0 || c == last_cloud) {
      RS_MSG << "t=" << (int)dt << "s clouds=" << c << " images=" << n_image << " imu=" << n_imu
             << " sent_clouds=" << n_sent << " clients=" << g_clients.load() << " errors=" << n_err << RS_REND;
    }
    last_cloud = c;
  }
  driver.stop();
  g_cv.notify_all();
  if (net.joinable()) net.detach();  // accept() may be blocking; process exit tears it down
#ifdef _WIN32
  WSACleanup();
#endif
  RS_TITLE << "stopped: clouds=" << n_cloud << " images=" << n_image << " imu=" << n_imu << " errors=" << n_err << RS_REND;
  return 0;
}
