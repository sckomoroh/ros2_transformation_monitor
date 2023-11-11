#pragma once

#include <memory>

#include <QAbstractItemModel>
#include <QDebug>

#include "node/MonitorNode.h"

namespace tf::tools::ui::models {

struct Frame {
    using SharedPtr = std::shared_ptr<Frame>;

    QString name;
    Frame* parent;
    std::list<Frame::SharedPtr> children;

    bool operator==(const Frame& frame) const { return name == frame.name; }
};

class FrameTreeModel : public QAbstractItemModel {
    Q_OBJECT
private:
    Frame::SharedPtr m_rootFrame;

public:
    FrameTreeModel(QObject* parent = nullptr);
    ~FrameTreeModel();

public:
    void updateModel(const std::shared_ptr<node::Frame>& frame);

public:  // QAbstractItemModel
    QModelIndex index(int row,
                      int column,
                      const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& child) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;

    Qt::ItemFlags flags(const QModelIndex& index) const override;

private:
    Frame* getItem(const QModelIndex& index) const;
    void copyTree(Frame::SharedPtr item, const std::shared_ptr<node::Frame>& frame);
};

}  // namespace tf::tools::ui::models
