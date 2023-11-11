#include "FrameTreeModel.h"

namespace tf::tools::ui::models {

FrameTreeModel::FrameTreeModel(QObject* parent)
    : QAbstractItemModel(parent)
    , m_rootFrame(new Frame)
{
    m_rootFrame->name = "MODEL_ROOT";
}

FrameTreeModel::~FrameTreeModel() {}

void FrameTreeModel::updateModel(const std::shared_ptr<node::Frame>& frame)
{
    beginResetModel();

    m_rootFrame->children.clear();
    copyTree(m_rootFrame, frame);

    endResetModel();
}

QModelIndex FrameTreeModel::index(int row, int column, const QModelIndex& parent) const
{
    if (!hasIndex(row, column, parent))
        return QModelIndex();

    Frame* parentFrame;

    if (!parent.isValid()) {
        parentFrame = m_rootFrame.get();
    }
    else {
        parentFrame = getItem(parent);
    }

    auto childFrame = std::next(parentFrame->children.begin(), row);

    if (childFrame != parentFrame->children.end()) {
        return createIndex(row, column, childFrame->get());
    }

    return QModelIndex();
}

QModelIndex FrameTreeModel::parent(const QModelIndex& index) const
{
    if (!index.isValid()) {
        return QModelIndex();
    }

    Frame* childFrame = getItem(index);
    Frame* parentFrame = childFrame->parent;

    if (parentFrame == m_rootFrame.get()) {
        return QModelIndex();
    }

    Frame* grandParentFrame = parentFrame->parent;
    int row = std::distance(
        grandParentFrame->children.begin(),
        std::find_if(grandParentFrame->children.begin(), grandParentFrame->children.end(),
                     [parentFrame](Frame::SharedPtr item) { return item.get() == parentFrame; }));

    return createIndex(row, 0, parentFrame);
}

int FrameTreeModel::rowCount(const QModelIndex& parent) const
{
    if (parent.column() > 0) {
        return 0;
    }

    Frame* parentFrame = nullptr;
    if (!parent.isValid()) {
        parentFrame = m_rootFrame.get();
    }
    else {
        parentFrame = getItem(parent);
    }

    return parentFrame ? parentFrame->children.size() : 0;
}

int FrameTreeModel::columnCount(const QModelIndex& parent) const
{
    Q_UNUSED(parent);
    return 1;  // Only name column
}

QVariant FrameTreeModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid()) {
        return QVariant();
    }

    if (role != Qt::DisplayRole) {
        return QVariant();
    }

    Frame* item = getItem(index);
    return item->name;
}

Qt::ItemFlags FrameTreeModel::flags(const QModelIndex& index) const
{
    if (!index.isValid()) {
        return Qt::NoItemFlags;
    }

    return Qt::ItemIsEditable | QAbstractItemModel::flags(index);
}

Frame* FrameTreeModel::getItem(const QModelIndex& index) const
{
    if (index.isValid()) {
        Frame* item = static_cast<Frame*>(index.internalPointer());
        if (item != nullptr) {
            return item;
        }
    }

    return m_rootFrame.get();
}

void FrameTreeModel::copyTree(Frame::SharedPtr item, const std::shared_ptr<node::Frame>& frame)
{
    auto childFrame = std::make_shared<Frame>();
    childFrame->name = frame->frame_id.c_str();
    childFrame->parent = item.get();
    for (auto& frameItem : frame->children) {
        copyTree(childFrame, frameItem);
    }

    item->children.push_back(childFrame);
}

}  // namespace tf::tools::ui::models
