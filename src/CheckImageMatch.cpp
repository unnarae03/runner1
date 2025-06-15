#include <behaviortree_cpp_v3/action_node.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>

using namespace std;
using namespace cv;
using json = nlohmann::json;
namespace fs = std::filesystem;

int loop_counter = 1;  // 🔢 전역 카운터 추가

class CheckImageMatch : public BT::SyncActionNode {
public:
  CheckImageMatch(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("code")
    };
  }

  BT::NodeStatus tick() override {
    std::string code = std::to_string(loop_counter++);  // 🔢 자동 증가 코드 사용

    std::string cur = "logs/images/current/" + code + ".jpg";
    std::string ref = "logs/images/reference/" + code + ".jpg";
    std::string out_img = "logs/images/result/" + code + ".jpg";
    std::string out_json = "logs/images/result/" + code + ".json";
    std::string out_bbox = out_img.substr(0, out_img.find(".jpg")) + "_bbox.jpg";

    Mat img1 = imread(cur, IMREAD_GRAYSCALE);
    Mat img2 = imread(ref, IMREAD_GRAYSCALE);
    if (img1.empty() || img2.empty()) {
      std::cerr << "[CheckImageMatch] 이미지 로딩 실패" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    Ptr<ORB> orb = ORB::create();
    vector<KeyPoint> kp1, kp2;
    Mat des1, des2;
    orb->detectAndCompute(img1, noArray(), kp1, des1);
    orb->detectAndCompute(img2, noArray(), kp2, des2);
    if (des1.empty() || des2.empty()) return BT::NodeStatus::FAILURE;

    BFMatcher bf(NORM_HAMMING, true);
    vector<DMatch> matches;
    bf.match(des1, des2, matches);

    double score = (double)matches.size() / max((int)kp1.size(), 1);
    bool anomaly = score < 0.4;

    Mat match_img;
    drawMatches(img1, kp1, img2, kp2, matches, match_img);
    imwrite(out_img, match_img);

    json j;
    j["match_score"] = score;
    j["anomaly"] = anomaly;
    j["image_path"] = out_img;
    j["cur_path"] = cur;
    j["ref_path"] = ref;
    j["bbox_path"] = out_bbox;

    ofstream o(out_json);
    o << j.dump(4);

    if (anomaly) {
      vector<Point2f> points;
      for (int i = 0; i < min(20, (int)matches.size()); ++i)
        points.push_back(kp2[matches[i].trainIdx].pt);
      if (!points.empty()) {
        Rect box = boundingRect(points);
        Mat color_img = imread(cur);
        rectangle(color_img, box, Scalar(0, 255, 0), 2);
        imwrite(out_bbox, color_img);
        cout << "[✓] Bounding box saved: " << out_bbox << endl;
      }
    }

    std::cout << "[✓] Processed: " << cur << " with score=" << score << std::endl;
    return anomaly ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

// ✅ 자동 비교 monitor 함수 (옵션)
void monitor() {
  while (true) {
    for (const auto& entry : fs::directory_iterator("logs/images/current")) {
      if (entry.path().extension() != ".jpg") continue;
      string base = entry.path().stem().string();
      string cur = "logs/images/current/" + base + ".jpg";
      string ref = "logs/images/reference/" + base + ".jpg";
      string out_img = "logs/images/result/" + base + ".jpg";
      string out_json = "logs/images/result/" + base + ".json";
      string out_bbox = out_img.substr(0, out_img.find(".jpg")) + "_bbox.jpg";

      if (!fs::exists(out_json) && fs::exists(ref)) {
        Mat img1 = imread(cur, IMREAD_GRAYSCALE);
        Mat img2 = imread(ref, IMREAD_GRAYSCALE);
        if (img1.empty() || img2.empty()) continue;

        Ptr<ORB> orb = ORB::create();
        vector<KeyPoint> kp1, kp2;
        Mat des1, des2;
        orb->detectAndCompute(img1, noArray(), kp1, des1);
        orb->detectAndCompute(img2, noArray(), kp2, des2);
        if (des1.empty() || des2.empty()) continue;

        BFMatcher bf(NORM_HAMMING, true);
        vector<DMatch> matches;
        bf.match(des1, des2, matches);

        double score = (double)matches.size() / max((int)kp1.size(), 1);
        bool anomaly = score < 0.4;

        Mat match_img;
        drawMatches(img1, kp1, img2, kp2, matches, match_img);
        imwrite(out_img, match_img);

        json j;
        j["match_score"] = score;
        j["anomaly"] = anomaly;
        j["image_path"] = out_img;
        j["cur_path"] = cur;
        j["ref_path"] = ref;
        j["bbox_path"] = out_bbox;

        ofstream o(out_json);
        o << j.dump(4);

        if (anomaly) {
          vector<Point2f> points;
          for (int i = 0; i < min(20, (int)matches.size()); ++i)
            points.push_back(kp2[matches[i].trainIdx].pt);
          if (!points.empty()) {
            Rect box = boundingRect(points);
            Mat color_img = imread(cur);
            rectangle(color_img, box, Scalar(0, 255, 0), 2);
            imwrite(out_bbox, color_img);
          }
        }
      }
    }
    this_thread::sleep_for(chrono::seconds(1));
  }
}
