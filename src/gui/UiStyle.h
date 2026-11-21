#pragma once
#include <QApplication>
#include <QFont>
#include <QPalette>

// Shared, flat desktop controls. Map colours live separately in Theme.h.
namespace UiStyle {
inline void apply(QApplication& app) {
    app.setStyle("Fusion");
    app.setFont(QFont("Segoe UI", 10));
    QPalette palette;
    palette.setColor(QPalette::Window, QColor("#F3F2EE"));
    palette.setColor(QPalette::WindowText, QColor("#242424"));
    palette.setColor(QPalette::Base, QColor("#FFFFFF"));
    palette.setColor(QPalette::Text, QColor("#242424"));
    palette.setColor(QPalette::Button, QColor("#FFFFFF"));
    palette.setColor(QPalette::ButtonText, QColor("#242424"));
    palette.setColor(QPalette::Highlight, QColor("#DD693C"));
    palette.setColor(QPalette::HighlightedText, QColor("#242424"));
    palette.setColor(QPalette::Disabled, QPalette::Text, QColor("#888880"));
    palette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor("#888880"));
    app.setPalette(palette);
    app.setStyleSheet(QStringLiteral(R"(
        QWidget { font-family: 'Segoe UI'; font-size: 13px; color: #242424; }
        QMainWindow, QDialog { background: #F3F2EE; }
        QLabel { background: transparent; }
        QPushButton, QComboBox, QSpinBox, QLineEdit {
            background: #FFFFFF; border: 1px solid #D0CEC8;
            border-radius: 5px; padding: 5px 10px; min-height: 20px;
        }
        QPushButton:hover, QComboBox:hover { background: #E9E7E1; border-color: #888880; }
        QPushButton:pressed { background: #DDDAD2; }
        QPushButton:checked { background: #E5E1D8; border-color: #666660; font-weight: 600; }
        QPushButton:focus, QComboBox:focus, QSlider:focus, QLineEdit:focus {
            border: 1px solid #DD693C;
        }
        QPushButton[primary="true"], QPushButton#routeBtn, QPushButton#runBtn { background: #DD693C; border-color: #BB542A; }
        QPushButton[primary="true"]:hover, QPushButton#routeBtn:hover, QPushButton#runBtn:hover { background: #E2774B; }
        QPushButton[primary="true"]:pressed, QPushButton#routeBtn:pressed, QPushButton#runBtn:pressed { background: #BB542A; }
        QPushButton[primary="true"]:focus, QPushButton#routeBtn:focus, QPushButton#runBtn:focus { border: 2px solid #242424; padding: 4px 9px; }
        QPushButton[primary="true"]:disabled, QPushButton:disabled, QPushButton#routeBtn:disabled, QPushButton#runBtn:disabled {
            background: #ECEAE4; color: #888880; border: 1px solid #D0CEC8;
        }
        QComboBox::drop-down { width:24px; border-left:1px solid #D0CEC8; }
        QComboBox::down-arrow { image:url(:/controls/down.xpm); width:10px; height:6px; }
        QPushButton#handleBtn { padding:0; font-size:20px; border-radius:0; }
        QSpinBox { padding-right:28px; }
        QSpinBox::up-button, QSpinBox::down-button { width:24px; background:#ECEEEB; border-left:1px solid #D0CEC8; }
        QSpinBox::up-arrow { image:url(:/controls/up.xpm); width:10px; height:6px; }
        QSpinBox::down-arrow { image:url(:/controls/down.xpm); width:10px; height:6px; }
        QSpinBox::up-button:hover, QSpinBox::down-button:hover { background:#DDDAD2; }
        QSpinBox:focus { border-color:#DD693C; }
        QSpinBox:disabled, QComboBox:disabled { background:#ECEAE4; color:#888880; }
        QRadioButton { spacing:10px; padding:4px 0; }
        QRadioButton::indicator { width:14px; height:14px; border:1px solid #939C9F; border-radius:7px; background:#FFFFFF; }
        QRadioButton::indicator:checked { border:4px solid #DD693C; width:8px; height:8px; background:#FFFFFF; }
        QRadioButton::indicator:hover { border-color:#DD693C; }
        QRadioButton:focus { color:#A33F18; }
        QSlider::handle:horizontal:hover { border-color:#DD693C; }
        QSlider::handle:horizontal:pressed { background:#DD693C; }
        QSlider::handle:horizontal:disabled { background:#D0CEC8; border-color:#B1AEA5; }
        QComboBox QAbstractItemView { background: #FFFFFF; selection-background-color: #E5E1D8; selection-color: #242424; }
        QSlider { min-height: 24px; }
        QSlider::groove:horizontal { height: 4px; background: #D0CEC8; }
        QSlider::sub-page:horizontal { background: #666660; }
        QSlider::handle:horizontal { width: 12px; margin: -6px 0; background: #FFFFFF; border: 1px solid #666660; border-radius: 5px; }
        QScrollArea { border: none; background: transparent; }
        QScrollBar:vertical { width: 12px; background: #ECEAE4; }
        QScrollBar::handle:vertical { background: #B1AEA5; min-height: 24px; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        QProgressBar { border: 1px solid #D0CEC8; background: #ECEAE4; text-align: center; }
        QProgressBar::chunk { background: #666660; }
        QToolTip { background: #FFFFFF; color: #242424; border: 1px solid #D0CEC8; }
        QFrame#testRow { border: none; background: transparent; }
        QFrame#testRow:hover { background: #E9E7E1; }
    )"));
}
}
