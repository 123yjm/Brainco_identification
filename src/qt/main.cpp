#include "main_window.hpp"
#include <QApplication>

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  app.setApplicationName("identify_gui");

  MainWindow window;
  window.resize(800, 700);
  window.show();

  return app.exec();
}
