#ifndef SIMULATIONOBJECT_H
#define SIMULATIONOBJECT_H

#include <QPointF>
#include <QRectF>
#include "RoadGeometry.h"   // Direction i stałe geometrii

/**
 * @brief Abstrakcyjna klasa bazowa dla każdego obiektu symulacji.
 *
 * Definiuje wspólny interfejs odpytywany przez silnik i renderery: pozycję,
 * prostokąt kolizji, kąt obrotu oraz dane nawigacyjne (kierunek, droga,
 * prędkość). Klasa @ref Car jest jej główną implementacją.
 */
class SimulationObject
{
public:
    virtual ~SimulationObject() = default;

    virtual void    update(double dt)    = 0; ///< Krok aktualizacji obiektu.
    virtual QPointF position()     const = 0; ///< Pozycja (lewy-górny róg).
    virtual QRectF  boundingRect() const = 0; ///< Prostokąt kolizji/rysowania.

    virtual double headingAngle()      const { return 0.0;  } ///< Kąt obrotu (stopnie).
    virtual bool   isCircling(int,int) const { return false; } ///< Czy krąży na danym rondzie.
    virtual int    blinkerSide()       const { return 0;    } ///< Kierunkowskaz: 0/1/2.
    virtual bool   blinkerFlash()      const { return false; } ///< Faza migania kierunkowskazu.

    // ── Nawigacja / analiza ruchu ────────────────────────────────
    virtual Direction dir()       const { return Direction::L2R; } ///< Kierunek jazdy.
    virtual int       road()      const { return 0;              } ///< Numer drogi.
    virtual double    speed()     const { return 0.0;            } ///< Aktualna prędkość.
    virtual bool      isSmartCar()const { return false;          } ///< Czy to auto GPS.
    virtual bool      isOnRing()  const { return false;          } ///< Czy jest na rondzie.
    virtual int       ringVI()    const { return -1;             } ///< Indeks vi ronda (-1 = nie na rondzie).
    virtual int       ringHI()    const { return -1;             } ///< Indeks hi ronda (-1 = nie na rondzie).
};

#endif // SIMULATIONOBJECT_H
