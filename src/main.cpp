#include <rclcpp/rclcpp.hpp>

#include <QApplication>

#include "ui/MainWindow.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    QApplication app(argc, argv);

    tf::tools::ui::MainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}
