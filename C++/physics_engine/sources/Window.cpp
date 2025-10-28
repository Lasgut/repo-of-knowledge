#include "window.h"

Window::Window(QWidget *parent) 
    : QMainWindow(parent) {
    // Set window title
    setWindowTitle("Physics Engine Window");

    // Set default window size
    resize(800, 600);
}

Window::~Window() {
    // Any necessary cleanup (in this simple case, Qt will handle most)
}
