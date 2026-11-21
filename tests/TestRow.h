#pragma once
#include <QFrame>

class QLabel;
namespace Ui { class TestRow; }  ///< Generowane z ui/widgets/test_row.ui

/**
 * @brief Klikalny wiersz testu — SZABLON z Qt Designer.
 *
 * Układ wiersza (ikona statusu + nazwa testu, kursor, podpowiedź) jest
 * zdefiniowany w @c ui/widgets/test_row.ui. Kod ustawia nazwę testu,
 * przepuszczanie kliknięć przez etykiety (atrybut niedostępny w Designerze)
 * oraz objectName @c testRow, na który wskazuje selektor hover w QSS okna.
 * Klikalność zapewnia eventFilter instalowany przez @c TestWindow.
 */
class TestRow : public QFrame {
    Q_OBJECT
public:
    explicit TestRow(const QString& testName, QWidget* parent = nullptr);
    ~TestRow() override;

    QLabel* icon()  const;   ///< Ikona statusu (aktualizowana przez renderTestRow).
    QLabel* label() const;   ///< Etykieta z nazwą testu.

private:
    Ui::TestRow* ui = nullptr;
};
