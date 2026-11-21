/**
 * @file test_main.cpp
 * @brief Punkt wejścia aplikacji testów jednostkowych EasyRider.
 *
 *  Domyślnie uruchamia interfejs graficzny (@ref TestWindow). Z flagą
 *  @c --headless [N] (lub @c --run-all [N]) uruchamia bezgłowy wykonawca
 *  @ref HeadlessRunner — wykonuje N przebiegów wszystkich testów
 *  (domyślnie 1) i kończy proces kodem 0 (wszystko zaliczone) lub 1.
 *  Tryb headless wymusza platformę Qt „offscreen", więc działa bez X-serwera.
 */
#include <QApplication>
#include "gui/UiStyle.h"
#include <QString>
#include "TestWindow.h"
#include "HeadlessRunner.h"
#include "audit/MechanicsAudit.h"

int main(int argc, char* argv[]) {
    // Wykryj tryb headless ZANIM powstanie QApplication, aby zdążyć wymusić
    // platformę offscreen (inaczej Qt próbowałoby otworzyć okno na ekranie).
    bool headless = false;
    int  repeats  = 1;
    bool audit    = false;
    QString auditFilter;
    for (int i = 1; i < argc; ++i) {
        const QString a = QString::fromLocal8Bit(argv[i]);
        if (a == QStringLiteral("--audit")) {
            audit = headless = true;
            if (i + 1 < argc) auditFilter = QString::fromLocal8Bit(argv[++i]);
            continue;
        }
        if (a == QStringLiteral("--headless") || a == QStringLiteral("--run-all")) {
            headless = true;
            if (i + 1 < argc) {
                bool ok = false;
                const int n = QString::fromLocal8Bit(argv[i + 1]).toInt(&ok);
                if (ok && n > 0) { repeats = n; ++i; }
            }
        }
    }

    if (headless) qputenv("QT_QPA_PLATFORM", "offscreen");

    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("EasyRider Tests"));

    if (audit)
        return Audit::runCli(auditFilter);
    if (headless)
        return HeadlessRunner::runAll(repeats);

    UiStyle::apply(app);
    TestWindow window;
    window.show();
    return app.exec();
}
