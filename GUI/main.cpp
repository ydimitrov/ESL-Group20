#include "cockpit.h"
#include <QApplication>
#include <QPalette>
#include <QStyleFactory>
#include <QtSerialPort/QSerialPort>


void configureDarkStyle()
{
    qApp->setStyle (QStyleFactory::create ("Fusion"));
    QPalette darkPalette;
    darkPalette.setColor (QPalette::BrightText,      Qt::red);
    darkPalette.setColor (QPalette::WindowText,      Qt::white);
    darkPalette.setColor (QPalette::ToolTipBase,     Qt::white);
    darkPalette.setColor (QPalette::ToolTipText,     Qt::white);
    darkPalette.setColor (QPalette::Text,            Qt::white);
    darkPalette.setColor (QPalette::ButtonText,      Qt::white);
    darkPalette.setColor (QPalette::HighlightedText, Qt::black);
    darkPalette.setColor (QPalette::Window,          QColor (53, 53, 53));
    darkPalette.setColor (QPalette::Base,            QColor (25, 25, 25));
    darkPalette.setColor (QPalette::AlternateBase,   QColor (53, 53, 53));
    darkPalette.setColor (QPalette::Button,          QColor (53, 53, 53));
    darkPalette.setColor (QPalette::Link,            QColor (42, 130, 218));
    darkPalette.setColor (QPalette::Highlight,       QColor (42, 130, 218));
    qApp->setPalette (darkPalette);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    configureDarkStyle();
    CockPit w;

    w.show();
    return a.exec();
}
