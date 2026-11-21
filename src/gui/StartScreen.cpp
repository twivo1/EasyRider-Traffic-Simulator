#include "StartScreen.h"
#include "SimulationView.h"
#include "MapRenderer.h"
#include "WorldView.h"
#include "../core/SimulationEngine.h"
#include "../core/RoadGeometry.h"
#include <QPainter>
#include <QStackedWidget>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <random>
#include <cmath>

static std::mt19937 sRng(std::random_device{}());

StartScreen::StartScreen(SimulationEngine* engine, SimulationView* view, QWidget* parent)
    : QWidget(parent), m_engine(engine), m_view(view), m_themes(allThemes()) {
    setAttribute(Qt::WA_TranslucentBackground);
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);
    auto* header = new QLabel("EasyRider  /  Symulator ruchu drogowego", this);
    header->setStyleSheet("background:#FFFFFF; border-bottom:1px solid #D0CEC8; padding:10px 16px; font-weight:600;");
    root->addWidget(header);
    m_pages = new QStackedWidget(this);
    root->addWidget(m_pages, 1);

    auto button = [](const QString& text, const char* name, QBoxLayout* layout) {
        auto* b = new QPushButton(text);
        b->setObjectName(name);
        b->setProperty("primary", QStringList{"beginButton","themeNext","acceptMap","startButton"}.contains(name));
        b->setMinimumHeight(32);
        layout->addWidget(b);
        return b;
    };
    auto form = [this](const QString& title) {
        auto* page = new QWidget;
        auto* outer = new QVBoxLayout(page);
        outer->setContentsMargins(16, 16, 16, 16);
        outer->addStretch();
        auto* body = new QWidget;
        body->setMaximumWidth(400);
        auto* layout = new QVBoxLayout(body);
        layout->setContentsMargins(0,0,0,0);
        layout->setSpacing(12);
        auto* heading = new QLabel(title);
        heading->setStyleSheet("font-size:18px; font-weight:600; margin-bottom:8px;");
        layout->addWidget(heading);
        outer->addWidget(body, 0, Qt::AlignHCenter);
        outer->addStretch();
        m_pages->addWidget(page);
        return layout;
    };
    auto* title = form("EasyRider");
    auto* intro = new QLabel("Symulacja ruchu na siatce dróg.\nSkrzyżowania, ronda i nawigacja A*.");
    intro->setStyleSheet("color:#677075;");
    title->addWidget(intro);
    connect(button("Rozpocznij", "beginButton", title), &QPushButton::clicked,
            this, [this] { m_view->setTheme(m_themes[m_themeIdx]); setPage(Page::ThemePick); });
    connect(button("Wyjście", "exitButton", title), &QPushButton::clicked, this, [this] { window()->close(); });

    auto* themes = form("1 / 3   Wygląd mapy");
    auto* group = new QButtonGroup(this);
    for (int i=0; i<(int)m_themes.size(); ++i) {
        auto* row = new QHBoxLayout;
        auto* choice = new QRadioButton(m_themes[i].name);
        choice->setObjectName(QString("themeChoice%1").arg(i));
        choice->setChecked(i == m_themeIdx);
        group->addButton(choice, i);
        row->addWidget(choice, 1);
        for (QColor c : {m_themes[i].grassTop, m_themes[i].road, m_themes[i].carBody}) {
            auto* swatch = new QLabel;
            swatch->setFixedSize(22, 16);
            swatch->setStyleSheet(QString("background:%1; border:1px solid #999C96;").arg(c.name()));
            row->addWidget(swatch);
        }
        themes->addLayout(row);
    }
    connect(group, &QButtonGroup::idClicked, this, [this](int i) {
        m_themeIdx=i; m_view->setTheme(m_themes[i]);
    });
    auto* themeActions = new QHBoxLayout; themes->addLayout(themeActions);
    connect(button("Wróć", "themeBack", themeActions), &QPushButton::clicked, this, [this] { setPage(Page::Title); });
    connect(button("Dalej", "themeNext", themeActions), &QPushButton::clicked, this, [this] { generateNewMap(); setPage(Page::MapPreview); });

    auto* preview = new QWidget;
    auto* previewLayout = new QVBoxLayout(preview);
    previewLayout->setContentsMargins(0,0,0,0);
    previewLayout->addStretch();
    auto* footer = new QWidget;
    footer->setObjectName("mapPreviewFooter");
    footer->setStyleSheet("QWidget { background:#FFFFFF; }");
    auto* footerLayout = new QHBoxLayout(footer);
    footerLayout->setContentsMargins(12,8,12,8);
    m_mapInfo = new QLabel;
    footerLayout->addWidget(m_mapInfo, 1);
    connect(button("Wróć", "mapBack", footerLayout), &QPushButton::clicked, this, [this] { setPage(Page::ThemePick); });
    connect(button("Nowa mapa", "newMap", footerLayout), &QPushButton::clicked, this, [this] { generateNewMap(); setPage(Page::MapPreview); });
    connect(button("Akceptuj", "acceptMap", footerLayout), &QPushButton::clicked, this, [this] { setPage(Page::CarCount); });
    previewLayout->addWidget(footer);
    m_pages->addWidget(preview);

    // Keep the accepted map visible while configuring its traffic count.
    auto* countPage = new QWidget;
    auto* countLayout = new QVBoxLayout(countPage);
    countLayout->setContentsMargins(0,0,0,0);
    countLayout->addStretch();
    auto* countFooter = new QWidget;
    countFooter->setObjectName("carCountFooter");
    countFooter->setStyleSheet("QWidget#carCountFooter { background:#FFFFFF; }");
    auto* cars = new QHBoxLayout(countFooter);
    cars->setContentsMargins(12,8,12,8);
    cars->setSpacing(12);
    cars->addWidget(new QLabel("3 / 3   Natężenie ruchu"));
    cars->addStretch();
    auto* value = new QSpinBox;
    value->setObjectName("initialCarCount");
    value->setRange(1,100); value->setValue(m_carCount); value->setSingleStep(5);
    value->setSuffix(" pojazdów");
    value->setFixedWidth(150);
    cars->addWidget(value);
    auto* slider = new QSlider(Qt::Horizontal);
    slider->setObjectName("initialCarSlider");
    slider->setRange(1,100); slider->setValue(m_carCount);
    slider->setFixedWidth(180);
    cars->addWidget(slider);
    connect(slider,&QSlider::valueChanged,value,&QSpinBox::setValue);
    connect(value,&QSpinBox::valueChanged,slider,&QSlider::setValue);
    connect(value,&QSpinBox::valueChanged,this,[this](int n) { m_carCount=n; });
    cars->addSpacing(12);
    auto* carActions = cars;
    countLayout->addWidget(countFooter);
    m_pages->addWidget(countPage);
    connect(button("Wróć", "carBack", carActions), &QPushButton::clicked, this, [this] { setPage(Page::MapPreview); });
    connect(button("Uruchom symulację", "startButton", carActions), &QPushButton::clicked, this, [this] {
        hide(); setAttribute(Qt::WA_TransparentForMouseEvents, true);
        emit startSimulation(m_carCount,m_themes[m_themeIdx]);
    });
}

void StartScreen::setPage(Page page) {
    m_page=page;
    m_pages->setCurrentIndex(static_cast<int>(page));
    if (page==Page::MapPreview) {
        int rounds=0;
        for(int vi=0;vi<kNumRoads;++vi)
            for(int hi=0;hi<kNumRoads;++hi)
                if(m_engine->intersectionAt(vi,hi).isRoundabout) ++rounds;
        m_mapInfo->setText(QString("2 / 3   Układ mapy   •   Ronda: %1   •   Sygnalizacja: %2").arg(rounds).arg(kNumRoads*kNumRoads-rounds));
        m_view->update();
    }
    update();
}
void StartScreen::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(),QColor("#F3F2EE"));
    if (m_page==Page::MapPreview || m_page==Page::CarCount) {
        // Render the preview within its own available area, clear of wizard controls.
        const auto* footer=findChild<QWidget*>(m_page==Page::MapPreview ? "mapPreviewFooter" : "carCountFooter");
        const QRectF available=m_pages->geometry().adjusted(8,8,-8,-footer->height()-8);

        const MapTheme& theme=m_themes[m_themeIdx];
        p.fillRect(available,theme.grassTop);
        p.setRenderHint(QPainter::Antialiasing);
        p.setTransform(WorldView::fitted(available));
        MapRenderer renderer;
        renderer.drawRoads(p,theme);
        renderer.drawMarkings(p,theme);
        renderer.drawIntersections(p,m_engine,theme);
    }
}

void StartScreen::generateNewMap() {
    std::uniform_int_distribution<int> countDist(5, 10);
    std::uniform_int_distribution<int> posDist(0, kNumRoads - 1);
    int count = countDist(sRng);
    std::vector<std::pair<int,int>> rbs;
    for (int i = 0; i < count; i++) {
        for (int att = 0; att < 60; att++) {
            int vi = posDist(sRng), hi = posDist(sRng);
            bool ok = true;
            for (auto& [rv,rh] : rbs) {
                if (vi==rv && hi==rh) { ok=false; break; }
                if (std::abs(vi-rv)+std::abs(hi-rh)<=1) { ok=false; break; }
            }
            if (ok) { rbs.push_back({vi,hi}); break; }
        }
    }
    m_engine->setRoundabouts(rbs);
    if (m_view) m_view->update();
}
