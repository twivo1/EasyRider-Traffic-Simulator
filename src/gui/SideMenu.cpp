#include "SideMenu.h"
#include "ui_side_menu.h"   // wygenerowane przez uic z ui/side_menu.ui

#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalBlocker>
#include <QResizeEvent>
#include <algorithm>

static const double kSpeedTable[] = {0.25,0.5,0.75,1.0,1.5,2.0,3.0,4.0};

SideMenu::SideMenu(QWidget* parent)
    : QWidget(parent), ui(new Ui::SideMenu) {

    buildUI();
}

SideMenu::~SideMenu() {
    delete ui;   // zwolnij obiekt widżetów z .ui (same widżety usuwa Qt)
}

void SideMenu::buildUI() {
    // Całe drzewo kontrolek (uchwyt, panel, suwak prędkości, lista motywów,
    // przyciski, etykiety, separatory) pochodzi z Qt Designer — ui/side_menu.ui.
    // setupUi tworzy je wraz z arkuszami stylów (QSS) osadzonymi w .ui.
    ui->setupUi(this);
    bindFromUi();

    // ── Sygnały kontrolek (logika bez zmian) ─────────────────────
    connect(m_handleBtn, &QPushButton::clicked, this, &SideMenu::toggle);

    connect(m_speedSlider, &QSlider::valueChanged, this, [this](int v){
        ui->speedLabel->setText(QString("Prędkość symulacji · %1×").arg(kSpeedTable[std::clamp(v-1,0,7)]));
        emit speedChanged(kSpeedTable[std::clamp(v-1,0,7)]);
    });

    connect(ui->heatBtn, &QPushButton::toggled, this, &SideMenu::heatmapToggled);

    // Lista motywów — wypełniana DYNAMICZNIE z dostępnych motywów
    // (treść zależna od danych pozostaje w kodzie).
    m_themes = allThemes();
    for (const auto& th : m_themes)
        m_themeCombo->addItem(th.name);
    connect(m_themeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx){
        if (idx >= 0 && idx < (int)m_themes.size())
            emit themeChanged(m_themes[idx]);
    });

    connect(m_routeBtn,  &QPushButton::clicked, this, &SideMenu::routeRequested);
    connect(m_cancelBtn, &QPushButton::clicked, this, &SideMenu::cancelRouteRequested);
    connect(ui->backBtn, &QPushButton::clicked, this, &SideMenu::backToMenuRequested);

    reposition();
}

// Powiązanie wskaźników składowych z widżetami utworzonymi przez setupUi().
void SideMenu::bindFromUi() {
    m_handleBtn   = ui->handleBtn;
    m_panel       = ui->sidePanel;
    m_routeBtn    = ui->routeBtn;
    m_cancelBtn   = ui->cancelBtn;
    m_carCount    = ui->carCount;
    m_stats       = ui->stats;
    m_speedSlider = ui->speedSlider;
    m_themeCombo  = ui->themeCombo;
}

// ── Przeliczenie geometrii panelu ────────────────────────────────────
void SideMenu::reposition() {
    setFixedWidth(m_open ? kPanelWidth+kHandleW : kHandleW);
    ui->panelScroll->setVisible(m_open);
    m_handleBtn->setText(m_open ? "›" : "‹");
    m_handleBtn->setAccessibleName(m_open ? "Ukryj sterowanie" : "Pokaż sterowanie");
}

// Panel occupies its own layout column; collapsing returns space to the map.
void SideMenu::toggle() {
    m_open = !m_open;
    reposition();

}

// ── Publiczne metody aktualizujące ───────────────────────────────────
void SideMenu::setCarCount(int n) {
    m_carCount->setText(QString("Pojazdy na mapie: %1").arg(n));
}

void SideMenu::setSelectedTheme(const MapTheme& theme) {
    const QSignalBlocker blocker(m_themeCombo);
    for (int i=0; i<(int)m_themes.size(); ++i)
        if (m_themes[i].id == theme.id) m_themeCombo->setCurrentIndex(i);
}

void SideMenu::setNavStats(double cost, int recalcs, int hotspots, double estTime) {
    if (cost <= 0.0) { m_stats->clear(); return; }
    m_stats->setText(
        QString("Nawigacja aktywna\n"
                "Koszt: %1 s\n"
                "Przeliczeń: %2\n"
                "Korki: %3\n"
                "Czas ~%4 s")
            .arg(cost, 0, 'f', 1).arg(recalcs).arg(hotspots).arg((int)estTime));
}

void SideMenu::setRouteSelectionMode(bool active) {
    m_routeBtn->setEnabled(!active);
    m_cancelBtn->setVisible(active);
    m_routeBtn->setText(active
        ? "Wybierz start i cel..." 
        : "Wyznacz trasę");         

    // The handle is outside the map, so cancellation remains accessible.
    m_handleBtn->setVisible(true);
}
