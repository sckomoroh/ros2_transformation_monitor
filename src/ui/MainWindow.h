#pragma once

#include <QDialog>
#include <QItemSelection>

#include "models/FrameTreeModel.h"
#include "node/MonitorNode.h"

namespace Ui {
class MainWindow;
}

namespace tf::tools::ui {

class MainWindow final
    : public QDialog
    , public node::MonitorNodeListener {
    Q_OBJECT
private:
    Ui::MainWindow* mUi;
    std::shared_ptr<tf::tools::node::MonitorNode> mNode;
    models::FrameTreeModel mModel;
    QItemSelectionModel* mSelectionModel;

public:
    MainWindow();
    ~MainWindow();

public:  // MonitorNodeListener
    void onFramesTreeUpdated() override;
    void onFrameTransformationUpdated() override;

signals:
    void framesTreeUpdated();
    void frameUpdated();

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void on_framesTreeUpdated();
    void on_selectionChanged(const QItemSelection& selected, const QItemSelection& deselected);
    void on_frameUpdated();
    void on_publishTransformClicked();

private:
    void updateTransformation(node::Frame::SharedPtr frame);
    void updateCalculatedTransformation(node::Frame::SharedPtr frame);
};

}  // namespace tf::tools::ui
