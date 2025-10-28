#include <QApplication>
#include "window.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Create the window object
    Window window;
    window.show();  // Show the window

    return app.exec();  // Run the application loop
}
