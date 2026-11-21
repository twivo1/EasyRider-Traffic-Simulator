#include "TestCategoryCard.h"
#include "ui_test_category_card.h"  // wygenerowane przez uic

TestCategoryCard::TestCategoryCard(const QString& name, const QString& accent,
                                   int count, QWidget* parent)
    : QFrame(parent), ui(new Ui::TestCategoryCard) {
    ui->setupUi(this);

    // Dane kategorii — treść i kolory akcentu nakładane na szablon z .ui.
    ui->catName->setText(name.toUpper());
    ui->catDot->hide();
    ui->catCount->setText(QString::number(count));
    ui->catCount->setStyleSheet(QString(
        "color:%1; background:%2; padding:2px 9px;"
        "font-size:10px; font-weight:700; border-radius:2px;"
        "border:1px solid %3;")
        .arg("#666660", "#F8F7F4", "#D0CEC8"));
}

TestCategoryCard::~TestCategoryCard() { delete ui; }

void TestCategoryCard::addRow(QWidget* row) {
    ui->rowsLayout->addWidget(row);
}
