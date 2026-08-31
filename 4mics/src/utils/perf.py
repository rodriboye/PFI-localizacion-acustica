"""
perf.py — Medición de tiempos del lazo de tiempo real.

El lazo de main.py tiene un presupuesto duro por frame (HOP_SIZE/SAMPLE_RATE =
23.2 ms). Si una iteración tarda más, el hilo lector no vacía su cola y se
descartan frames — en silencio, y sin que `pkts_lost` lo refleje, porque el
paquete si se leyo y la secuencia del contador del ESP32 queda intacta. Este
módulo existe para que ese modo de falla sea medible en vez de inferido.

    p50  caso típico, para dimensionar
    p95  el que decide: basta 1 de cada 20 frames por encima para que la cola se
         llene en ráfagas
    over fracción de iteraciones sobre el presupuesto
    rt   cómputo acumulado / duración del audio. >1.0 pierde frames si o si, y
         es independiente del tamaño del buffer: ningún encolado lo arregla

Uso:
    meter = PerfMeter("MUSIC", budget_ms=23.2)
    with meter:
        result = engine.process(frame)
    print(meter.summary())
"""

import time

import numpy as np


class PerfMeter:
    """Acumulador de tiempos con percentiles. Un perf_counter() y un append por
    muestra (~0.2 us), tres órdenes por debajo de lo que mide."""

    def __init__(self, name, budget_ms=None, capacity=500_000):
        self.name      = name
        self.budget_ms = budget_ms
        self.capacity  = capacity
        self._samples  = []       # segundos
        self._t0       = None
        self.dropped_samples = 0  # muestras no guardadas por tope de capacidad

    # --- context manager -------------------------------------------------
    def __enter__(self):
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *exc):
        self.add(time.perf_counter() - self._t0)
        return False   # nunca traga excepciones

    # --- API manual ------------------------------------------------------
    def add(self, dt_s):
        if len(self._samples) < self.capacity:
            self._samples.append(dt_s)
        else:
            self.dropped_samples += 1

    def reset(self):
        self._samples.clear()
        self.dropped_samples = 0

    # --- lectura ---------------------------------------------------------
    @property
    def n(self):
        return len(self._samples)

    @property
    def total_s(self):
        return float(np.sum(self._samples)) if self._samples else 0.0

    def stats(self, limit=None):
        """Tiempos en MILISEGUNDOS, o None si no hay datos.

        El display en vivo DEBE pasar un `limit`: sin el, el costo crece con la
        duración de la corrida y llega a ser comparable al del motor DOA. El
        resumen final usa la serie completa, pero se llama una sola vez.
        """
        src = self._samples if limit is None else self._samples[-int(limit):]
        if not src:
            return None
        a = np.asarray(src) * 1e3
        # Los tres percentiles en UNA llamada: domina el overhead de numpy.
        p50, p95, p99 = np.percentile(a, [50, 95, 99])
        d = {
            'n':    int(len(self._samples)),   # total real, no el de la ventana
            'mean': float(a.mean()),
            'p50':  float(p50),
            'p95':  float(p95),
            'p99':  float(p99),
            'max':  float(a.max()),
        }
        if self.budget_ms:
            d['over_pct'] = float(100.0 * np.mean(a > self.budget_ms))
        return d

    def realtime_factor(self, audio_seconds):
        """Cómputo acumulado / duración del audio. >1.0 = no sostiene tiempo real."""
        if audio_seconds <= 0:
            return float('nan')
        return self.total_s / audio_seconds

    def summary(self, audio_seconds=None):
        s = self.stats()
        if s is None:
            return f"  {self.name:<22} (sin muestras)"
        txt = (f"  {self.name:<22} n={s['n']:<7d} "
               f"p50={s['p50']:6.2f}  p95={s['p95']:6.2f}  "
               f"p99={s['p99']:6.2f}  max={s['max']:7.2f}  ms")
        if self.budget_ms:
            flag = "  <-- EXCEDE" if s['over_pct'] > 5.0 else ""
            txt += (f"\n  {'':<22} presupuesto {self.budget_ms:.1f} ms  "
                    f"-> {s['over_pct']:.1f}% de los frames por encima{flag}")
        if audio_seconds:
            rt = self.realtime_factor(audio_seconds)
            # Mismos cortes que el VEREDICTO de main.py: si divergen, el reporte
            # se contradice consigo mismo.
            veredicto = ("OK, sostiene tiempo real" if rt < 0.85 else
                         "AL LIMITE" if rt < 1.0 else
                         "NO SOSTIENE TIEMPO REAL")
            txt += (f"\n  {'':<22} factor de tiempo real = {rt:.2f}x  "
                    f"({veredicto})")
        if self.dropped_samples:
            txt += f"\n  {'':<22} ({self.dropped_samples} muestras no registradas por tope)"
        return txt
