#ifndef WINDOW_H
#define WINDOW_H

#include <QMainWindow>

class Window : public QMainWindow {
    Q_OBJECT  // Qt's macro to enable signals and slots

public:
    Window(QWidget *parent = nullptr);  // Constructor
    ~Window();                          // Destructor
};

#endif // WINDOW_H
