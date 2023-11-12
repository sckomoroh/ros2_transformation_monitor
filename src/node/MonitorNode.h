#pragma once

#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace tf::tools::node {

struct Orientation {
    double pitch = 0.0;
    double roll = 0.0;
    double yaw = 0.0;

    Orientation& operator+(Orientation& other)
    {
        pitch += other.pitch;
        roll += other.roll;
        yaw += other.yaw;

        return *this;
    }
};

struct Position {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Position& operator+(Position& other)
    {
        x += other.x;
        y += other.y;
        z += other.z;

        return *this;
    }
};

struct Transform {
    Position position;
    Orientation orientation;

    Transform& operator+(Transform& other)
    {
        position = other.position + position;
        orientation = other.orientation + orientation;
        return *this;
    }
};

struct Frame : public std::enable_shared_from_this<Frame> {
    using SharedPtr = std::shared_ptr<Frame>;

    std::string frame_id;
    Transform transform;
    std::weak_ptr<Frame> parent;
    std::list<Frame::SharedPtr> children;

    void appendChild(const Frame::SharedPtr& frame)
    {
        children.push_back(frame);
        frame->parent = shared_from_this();
    }
};

class MonitorNodeListener {
public:
    virtual ~MonitorNodeListener() = default;

public:
    virtual void onFramesTreeUpdated() = 0;
    virtual void onFrameTransformationUpdated() = 0;
};

class MonitorNode : public rclcpp::Node {
private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr mTfSubscription;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr mTfStaticSubscription;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr mTfPublisher;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr mTfStaticPublisher;
    Frame::SharedPtr mRootFrame;
    bool mRun = true;
    std::thread mThread;
    MonitorNodeListener* mCallback = nullptr;
    std::mutex mMutex;

public:
    MonitorNode();
    ~MonitorNode();

public:
    void startNode();
    void stopNode();
    void setCallback(MonitorNodeListener* callback);

    void publishTransform(const char* parent,
                          const char* child,
                          Position position,
                          Orientation orientation,
                          bool isStatic);

    Frame::SharedPtr getRootNode();
    Frame::SharedPtr getNode(const std::string& frame_id);

private:
    void onTransformation(tf2_msgs::msg::TFMessage::SharedPtr msg);

    void updateFramesTree(const std::string& frame_id,
                          const std::string& child_frame_id,
                          const Transform& transform);

    void nodeSpin();

    Frame::SharedPtr findParentFrame(const Frame::SharedPtr rootFrame, const std::string& frame_id);
};

}  // namespace tf::tools::node
