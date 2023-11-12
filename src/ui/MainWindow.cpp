#include "MainWindow.h"

#include <rclcpp/rclcpp.hpp>

#include <QCloseEvent>

#include "ui_MainWindow.h"

namespace tf::tools::ui {

MainWindow::MainWindow()
    : mUi{new Ui::MainWindow}
    , mNode{std::make_shared<tf::tools::node::MonitorNode>()}
{
    mUi->setupUi(this);

    mUi->tree_frames->setModel(&mModel);

    mNode->setCallback(this);
    mNode->startNode();

    mSelectionModel = mUi->tree_frames->selectionModel();

    connect(mSelectionModel, &QItemSelectionModel::selectionChanged, this,
            &MainWindow::on_selectionChanged);
    connect(this, &MainWindow::framesTreeUpdated, this, &MainWindow::on_framesTreeUpdated);
    connect(this, &MainWindow::frameUpdated, this, &MainWindow::on_frameUpdated);
    connect(mUi->button_publish_transform, &QPushButton::clicked, this,
            &MainWindow::on_publishTransformClicked);
}

MainWindow::~MainWindow() { delete mUi; }

void MainWindow::closeEvent(QCloseEvent* event)
{
    mNode->stopNode();
    rclcpp::shutdown();
    event->accept();
}

void MainWindow::onFramesTreeUpdated() { emit framesTreeUpdated(); }

void MainWindow::onFrameTransformationUpdated() { emit frameUpdated(); }

void MainWindow::on_framesTreeUpdated()
{
    auto rootNode = mNode->getRootNode();
    mModel.updateModel(rootNode);
}

void MainWindow::on_frameUpdated()
{
    //qDebug() << "On frames updates";
    auto itemSelected = mSelectionModel->selection();
    on_selectionChanged(itemSelected, itemSelected /*This parameter not used*/);
}

void MainWindow::on_selectionChanged(const QItemSelection& selected, const QItemSelection&)
{
    if (!selected.isEmpty()) {
        auto frame_id = mModel.data(selected.indexes().at(0), Qt::DisplayRole);
        auto frame = mNode->getNode(frame_id.toString().toStdString());
        if (frame->parent.lock() != nullptr) {
            mUi->button_publish_transform->setEnabled(true);
        }
        else {
            mUi->button_publish_transform->setEnabled(false);
        }

        updateTransformation(frame);
        updateCalculatedTransformation(frame);
    }
}

void MainWindow::on_publishTransformClicked()
{
    auto selected = mSelectionModel->selection();
    auto frame_id = mModel.data(selected.indexes().at(0), Qt::DisplayRole);
    auto frame = mNode->getNode(frame_id.toString().toStdString());
    auto parentFrame = frame->parent.lock();
    auto isStatic = mUi->check_static_transform->isChecked();

    node::Position position;
    node::Orientation orientation;

    position.x = mUi->line_transform_x->text().toDouble();
    position.y = mUi->line_transform_y->text().toDouble();
    position.z = mUi->line_transform_z->text().toDouble();

    orientation.roll = mUi->line_transform_roll->text().toDouble();
    orientation.pitch = mUi->line_transform_pitch->text().toDouble();
    orientation.yaw = mUi->line_transform_yaw->text().toDouble();

    mNode->publishTransform(parentFrame->frame_id.c_str(), frame->frame_id.c_str(), position,
                            orientation, isStatic);
}

void MainWindow::updateTransformation(node::Frame::SharedPtr frame)
{
    mUi->line_original_x->setText(QString::number(frame->transform.position.x, 'f', 2));
    mUi->line_original_y->setText(QString::number(frame->transform.position.y, 'f', 2));
    mUi->line_original_z->setText(QString::number(frame->transform.position.z, 'f', 2));
    mUi->line_original_roll->setText(QString::number(frame->transform.orientation.roll, 'f', 2));
    mUi->line_original_pitch->setText(QString::number(frame->transform.orientation.pitch, 'f', 2));
    mUi->line_original_yaw->setText(QString::number(frame->transform.orientation.yaw, 'f', 2));
}

void MainWindow::updateCalculatedTransformation(node::Frame::SharedPtr frame)
{
    double x, y, z, roll, pitch, yaw;
    x = frame->transform.position.x;
    y = frame->transform.position.y;
    z = frame->transform.position.z;
    roll = frame->transform.orientation.roll;
    pitch = frame->transform.orientation.pitch;
    yaw = frame->transform.orientation.yaw;
    auto parent = frame->parent.lock();
    while (parent != nullptr) {
        x += parent->transform.position.x;
        y += parent->transform.position.y;
        z += parent->transform.position.z;
        roll += parent->transform.orientation.roll;
        pitch += parent->transform.orientation.pitch;
        yaw += parent->transform.orientation.yaw;
        parent = parent->parent.lock();
    }

    mUi->line_x->setText(QString::number(x, 'f', 2));
    mUi->line_y->setText(QString::number(y, 'f', 2));
    mUi->line_z->setText(QString::number(z, 'f', 2));
    mUi->line_roll->setText(QString::number(roll, 'f', 2));
    mUi->line_pitch->setText(QString::number(pitch, 'f', 2));
    mUi->line_yaw->setText(QString::number(yaw, 'f', 2));
}

}  // namespace tf::tools::ui
