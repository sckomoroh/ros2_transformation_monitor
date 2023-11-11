#include "MonitorNode.h"

#include <cmath>

#include <algorithm>

#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace tf::tools::node {

MonitorNode::MonitorNode()
    : rclcpp::Node{"tf_tools_node"}
{
    using namespace std::placeholders;

    mTfSubscription = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 10, std::bind(&MonitorNode::onTransformation, this, _1));

    rclcpp::QoS qos{10};
    qos.history(rclcpp::HistoryPolicy::KeepAll);
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    qos.liveliness(rclcpp::LivelinessPolicy::Automatic);

    mTfStaticSubscription = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", qos, std::bind(&MonitorNode::onTransformation, this, _1));
}

MonitorNode::~MonitorNode()
{
    mRun = false;
    if (mThread.joinable()) {
        mThread.join();
    }
}

Frame::SharedPtr MonitorNode::getRootNode()
{
    std::unique_lock<std::mutex> lock{mMutex};
    return mRootFrame;
}

Frame::SharedPtr MonitorNode::getNode(const std::string& frame_id)
{
    std::unique_lock<std::mutex> lock{mMutex};
    return findParentFrame(mRootFrame, frame_id);
}

void MonitorNode::startNode() { mThread = std::thread{&MonitorNode::nodeSpin, this}; }

void MonitorNode::stopNode()
{
    mRun = false;
    if (mThread.joinable()) {
        mThread.join();
    }
}

void MonitorNode::setCallback(MonitorNodeListener* callback) { mCallback = callback; }

Orientation toRPY(const geometry_msgs::msg::Quaternion& q)
{
    Orientation orientation;

    tf2::Matrix3x3 matrix(tf2::Quaternion(q.x, q.y, q.z, q.w));
    matrix.getRPY(orientation.roll, orientation.pitch, orientation.yaw);

    orientation.roll *= 180 / M_PI;
    orientation.pitch *= 180 / M_PI;
    orientation.yaw *= 180 / M_PI;

    return orientation;
}

Position toPosition(geometry_msgs::msg::Vector3 pos)
{
    Position position;

    position.x = pos.x;
    position.y = pos.y;
    position.z = pos.z;

    return position;
}

void MonitorNode::onTransformation(tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    for (auto& transformItem : msg->transforms) {
        auto frame_id = transformItem.header.frame_id;
        auto child_frame_id = transformItem.child_frame_id;

        RCLCPP_DEBUG(get_logger(), "Catch transformation: %s -> %s", frame_id.c_str(),
                     child_frame_id.c_str());

        Transform transform;
        transform.orientation = toRPY(transformItem.transform.rotation);
        transform.position = toPosition(transformItem.transform.translation);

        updateFramesTree(frame_id, child_frame_id, transform);
    }
}

void MonitorNode::updateFramesTree(const std::string& frame_id,
                                   const std::string& child_frame_id,
                                   const Transform& transform)
{
    std::unique_lock<std::mutex> lock{mMutex};

    if (mRootFrame == nullptr) {
        mRootFrame = std::make_shared<Frame>();
        mRootFrame->frame_id = frame_id;
        mRootFrame->parent.reset();

        auto childFrame = std::make_shared<Frame>();
        childFrame->frame_id = child_frame_id;
        childFrame->transform = transform;
        mRootFrame->appendChild(childFrame);

        if (mCallback != nullptr) {
            mCallback->onFramesTreeUpdated();
        }

        return;
    }

    if (mRootFrame->frame_id == child_frame_id) {
        auto childFrame = mRootFrame;

        mRootFrame = std::make_shared<Frame>();
        mRootFrame->frame_id = frame_id;
        mRootFrame->parent.reset();
        childFrame->transform = transform;
        mRootFrame->appendChild(childFrame);
        if (mCallback != nullptr) {
            mCallback->onFramesTreeUpdated();
        }

        if (mCallback != nullptr) {
            mCallback->onFrameTransformationUpdated();
        }
    }

    auto parent = findParentFrame(mRootFrame, frame_id);
    if (parent != nullptr) {
        auto iter = std::find_if(parent->children.begin(), parent->children.end(),
                                 [&child_frame_id](const Frame::SharedPtr& item) {
                                     return item->frame_id == child_frame_id;
                                 });

        if (iter == parent->children.end()) {
            auto childFrame = std::make_shared<Frame>();
            childFrame->frame_id = child_frame_id;
            childFrame->transform = transform;
            parent->appendChild(childFrame);
            if (mCallback != nullptr) {
                mCallback->onFramesTreeUpdated();
            }
        }
        else {
            // (*iter)->transform = (*iter)->original_transform = transform;
            if (mCallback != nullptr) {
                mCallback->onFrameTransformationUpdated();
            }
        }
    }
}

void MonitorNode::nodeSpin()
{
    while (mRun) {
        rclcpp::spin_some(shared_from_this());
    }
}

Frame::SharedPtr MonitorNode::findParentFrame(const Frame::SharedPtr rootFrame,
                                              const std::string& frame_id)
{
    if (rootFrame->frame_id == frame_id) {
        return rootFrame;
    }

    for (auto child : rootFrame->children) {
        if (child->frame_id == frame_id) {
            return child;
        }

        auto frame = findParentFrame(child, frame_id);

        if (frame != nullptr) {
            return frame;
        }
    }

    return nullptr;
}

}  // namespace tf::tools::node
