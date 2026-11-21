#include "TestRow.h"
#include "ui_test_row.h"  // wygenerowane przez uic
#include <QLabel>

TestRow::TestRow(const QString& testName, QWidget* parent)
    : QFrame(parent), ui(new Ui::TestRow) {
    ui->setupUi(this);

    // Selektor QSS okna (QFrame#testRow / :hover) wskazuje na tę nazwę.
    setObjectName("testRow");

    ui->rowLabel->setText(testName);

    // Etykiety muszą przepuszczać kliknięcia do QFrame nadrzędnego —
    // tego atrybutu nie da się ustawić w Qt Designer, stąd w kodzie.
    ui->rowIcon->setAttribute(Qt::WA_TransparentForMouseEvents);
    ui->rowLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
}

TestRow::~TestRow() { delete ui; }

QLabel* TestRow::icon()  const { return ui->rowIcon; }
QLabel* TestRow::label() const { return ui->rowLabel; }
