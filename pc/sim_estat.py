"""Estatística de ensaios simulados: roda o sim.exe com N sementes por configuração e resume as métricas
como a tabela dos ensaios reais (mediana e faixa). A simulação é biestável, então um ensaio só não diz nada.

    python sim_estat.py [-n 8] [-d 30] "nome|secao.chave=valor ..." ["nome2|..." ...]
    ex.: python sim_estat.py "sem estimador|ensaio.estimador=0" "com estimador|"

Métricas extras: erro dθ2 = dθ2 que o controle viu - diferença central de θ2 (no topo); rug = rugosidade de θ2
(resíduo da 2ª diferença após ajustar a dinâmica do pêndulo, em LSB do AS5600). Nos ensaios reais:
sem estimador erro ~1,0 rad/s e rug 5-6; com estimador erro ~0,2 e rug 0,2-0,7.
"""
import os, subprocess, sys, tempfile
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import graficos as G

AQUI = os.path.dirname(os.path.abspath(__file__))
TS, Q = 0.02, 2 * np.pi / 4096


def rugosidade(D):
    a = np.clip(D["a"], -300, 300); th = D["theta2"]; v1 = D["dtheta1"]; k = np.arange(4, len(th) - 2)
    d2 = th[k + 1] - 2 * th[k] + th[k - 1]; d1 = (th[k + 1] - th[k - 1]) / (2 * TS)
    X = np.column_stack([a[k - j] * np.cos(th[k]) * TS**2 for j in range(-1, 4)] +
                        [np.sin(th[k]), d1, v1[k]**2 * np.sin(2 * th[k])])
    m = np.abs(D["w"][k]) < 0.35
    if m.sum() < 100:
        return np.nan, np.nan
    w, *_ = np.linalg.lstsq(X[m], d2[m], rcond=None)
    return (d2[m] - X[m] @ w).std() / Q, (D["dtheta2"][k] - d1)[m].std()


def roda(csv, extra, dur):
    subprocess.run([os.path.join(AQUI, "sim.exe"), os.path.join(AQUI, "furuta.ini"), csv, f"ensaio.duracao={dur}"]
                   + extra, cwd=AQUI, capture_output=True)
    D = G.le(csv); M = G.metricas(D)
    if M is not None:
        M["rug"], M["erro"] = rugosidade(D)
    return M


def main(argv):
    n, dur, casos = 8, 30, []
    it = iter(argv)
    for a in it:
        if a == "-n": n = int(next(it))
        elif a == "-d": dur = float(next(it))
        else: casos.append(a)
    if not casos:
        print(__doc__); return 2
    tmp = tempfile.mkdtemp(prefix="sim_estat_")
    for caso in casos:
        nome, _, extra = caso.partition("|")
        jobs = [(os.path.join(tmp, f"{abs(hash(nome))}_{s}.csv"), extra.split() + [f"sim.semente={s}"], dur)
                for s in range(1, n + 1)]
        with ThreadPoolExecutor(6) as ex:
            res = [r for r in ex.map(lambda j: roda(*j), jobs) if r]

        def f(k, fmt):
            v = np.array([r[k] for r in res])
            return f"{fmt.format(np.nanmedian(v))} ({fmt.format(np.nanmin(v))}–{fmt.format(np.nanmax(v))})" if len(v) else "–"
        print(f"{nome:18s} N={n} não subiu {n - len(res)} | 1ª passagem {f('subida', '{:.1f}')} s | captura {f('captura', '{:.1f}')} s | topo {f('topo', '{:.0f}')}% | quedas {f('quedas', '{:.0f}')} | "
              f"rms θ2 {f('rms_t2', '{:.1f}')}° | rms a {f('rms_a', '{:.0f}')} | erro dθ2 {f('erro', '{:.2f}')} | "
              f"rug {f('rug', '{:.1f}')}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
