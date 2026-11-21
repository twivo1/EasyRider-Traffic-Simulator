#pragma once
#include <QFrame>

class QWidget;
namespace Ui { class TestCategoryCard; }  ///< Generowane z ui/widgets/test_category_card.ui

/**
 * @brief Karta kategorii testów — SZABLON z Qt Designer.
 *
 * Wygląd karty (biała ramka, kropka koloru, nazwa, licznik, separator
 * oraz kontener wierszy @c rowsBox) jest zdefiniowany w pliku
 * @c ui/widgets/test_category_card.ui. Kod ustawia jedynie dane
 * (nazwa kategorii, kolor akcentu, liczba testów) i dosypuje dynamiczne
 * wiersze @ref TestRow przez @ref addRow().
 */
class TestCategoryCard : public QFrame {
    Q_OBJECT
public:
    /**
     * @param name   Nazwa kategorii (po polsku; wyświetlana wersalikami).
     * @param accent Kolor akcentu kategorii (np. "#16a34a").
     * @param count  Liczba testów w kategorii (badge po prawej).
     */
    explicit TestCategoryCard(const QString& name, const QString& accent,
                              int count, QWidget* parent = nullptr);
    ~TestCategoryCard() override;

    /// Dodaje wiersz testu do kontenera @c rowsBox z szablonu .ui.
    void addRow(QWidget* row);

private:
    Ui::TestCategoryCard* ui = nullptr;
};
