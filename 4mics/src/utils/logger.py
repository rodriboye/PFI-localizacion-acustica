"""
logger.py — Registro de eventos en CSV: UNA fila por EVENTO (no por frame).

Modo 'evento' (log_single):
    Fila inmediata con la PRIMERA estimación válida del evento (el ángulo del
    apuntado). duration_s = 0 y el rango colapsa al ángulo único.

Modo 'seguimiento' / sin servo (track + end_event):
    track() acumula las estimaciones del evento en curso sin escribir nada;
    al volver al silencio, end_event() escribe UNA fila con el RANGO de
    ángulos ocupado durante todo el seguimiento (min/max de azimut y
    elevación), la duración y los máximos de confianza/energía. close()
    cierra también un evento a medio registrar (Ctrl+C durante un
    seguimiento) para no perderlo.

Columnas:
    timestamp     inicio del evento (ISO-8601 local)
    mode          'evento' | 'seguimiento'
    duration_s    duración del registro del evento (s; 0.0 en modo evento)
    az_deg, el_deg              primera estimación válida del evento
    az_min_deg … el_max_deg     rango ocupado durante el evento
    conf_db       confianza máxima del evento (dB)
    energy_db     energía máxima del evento (dB)
"""

import csv
import math
import os
import time


class EventLogger:

    FIELDS = ['timestamp', 'mode', 'duration_s',
              'az_deg', 'el_deg',
              'az_min_deg', 'az_max_deg', 'el_min_deg', 'el_max_deg',
              'conf_db', 'energy_db']

    def __init__(self, filepath):
        self.filepath = filepath
        self._file    = None
        self._writer  = None
        self._tracking = None   # estado del evento en curso (modo seguimiento)
        self._open()

    def _open(self):
        new_file = not os.path.exists(self.filepath)
        self._file   = open(self.filepath, 'a', newline='')
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDS)
        if new_file:
            self._writer.writeheader()
            self._file.flush()

    # ------------------------------------------------------------ modo evento
    def log_single(self, doa_result, energy):
        """Registra de inmediato la primera estimación de un evento puntual."""
        if not doa_result or not doa_result.valid:
            return
        az, el = doa_result.azimuth, doa_result.elevation
        self._write_row(t0=time.time(), mode='evento', duration=0.0,
                        az=az, el=el, az_min=az, az_max=az,
                        el_min=el, el_max=el,
                        conf=doa_result.confidence, energy=energy)

    # ------------------------------------------------------- modo seguimiento
    def track(self, doa_result, energy):
        """Acumula una estimación del evento en curso (una por frame activo).
        No escribe nada: la fila única sale en end_event()."""
        if not doa_result or not doa_result.valid:
            return
        az, el = doa_result.azimuth, doa_result.elevation
        if self._tracking is None:
            self._tracking = {
                't0': time.time(), 'az0': az, 'el0': el,
                'az_min': az, 'az_max': az, 'el_min': el, 'el_max': el,
                'conf': doa_result.confidence, 'energy': energy,
            }
            return
        tr = self._tracking
        tr['az_min'] = min(tr['az_min'], az)
        tr['az_max'] = max(tr['az_max'], az)
        tr['el_min'] = min(tr['el_min'], el)
        tr['el_max'] = max(tr['el_max'], el)
        tr['conf']   = max(tr['conf'], doa_result.confidence)
        tr['energy'] = max(tr['energy'], energy)

    def end_event(self):
        """Cierra el evento en curso (si lo hay) y escribe SU única fila.
        Idempotente: llamarlo sin evento en curso no hace nada."""
        tr, self._tracking = self._tracking, None
        if tr is None:
            return
        self._write_row(t0=tr['t0'], mode='seguimiento',
                        duration=time.time() - tr['t0'],
                        az=tr['az0'], el=tr['el0'],
                        az_min=tr['az_min'], az_max=tr['az_max'],
                        el_min=tr['el_min'], el_max=tr['el_max'],
                        conf=tr['conf'], energy=tr['energy'])

    # ----------------------------------------------------------------- interno
    def _write_row(self, t0, mode, duration, az, el,
                   az_min, az_max, el_min, el_max, conf, energy):
        row = {
            'timestamp':  time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(t0)),
            'mode':       mode,
            'duration_s': round(duration, 2),
            'az_deg':     round(az, 2),
            'el_deg':     round(el, 2),
            'az_min_deg': round(az_min, 2),
            'az_max_deg': round(az_max, 2),
            'el_min_deg': round(el_min, 2),
            'el_max_deg': round(el_max, 2),
            'conf_db':    round(conf, 2),
            'energy_db':  round(10 * math.log10(max(energy, 1e-12)), 2),
        }
        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        self.end_event()   # no perder un seguimiento a medio registrar
        if self._file:
            self._file.close()
