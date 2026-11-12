"""Ajuste do estimador de dθ2 (estimador.c) com os ensaios reais.

O estimador prediz θ2, dθ2 com o modelo e a aceleração comandada e corrige com a inovação
(θ2 medido - previsto): θ2 += L1·inov, dθ2 += L2·inov. O filtro de Kalman em regime é o melhor preditor
de um passo, então os ganhos ótimos para os dados são os que minimizam a variância da inovação (método do
erro de predição): não precisa chutar as covariâncias de processo e de medida.

    python ajusta_estimador.py ensaios/ensaio_A.csv [...] [--atraso ms] [--ini furuta.ini]

Para cada ensaio: grade de (L1, L2) no trecho de equilíbrio e no ensaio inteiro, e para o ótimo:
inovação RMS (em LSB do AS5600), autocorrelação da inovação (≈ 0 = filtro bem ajustado), ruído do dθ2
(parte acima de ~6 Hz) e erro contra uma referência suave (derivada de θ2 filtrada sem atraso).
"""
import configparser
import sys

import numpy as np
from scipy.signal import butter, filtfilt

import graficos as G

TS, Q = 0.02, 2 * np.pi / 4096


def le_fisico(ini):
    cp = configparser.ConfigParser(inline_comment_prefixes=(";",), strict=False)
    cp.read(ini, encoding="utf-8")
    f = {k: float(v) for k, v in cp["fisico"].items()}
    de = f["i2"] + f["m2"] * f["l2"] ** 2
    return dict(de=de, be=f["m2"] * f["l2"] ** 2, ga=f["m2"] * f["l1"] * f["l2"], et=f["m2"] * f["g"] * f["l2"],
                mu2=f["mu2"])


def filtro(D, L1, L2, m, atraso_ms=0.0, nsub=10, atraso_med_ms=0.0, acel_real=False, L3=0.0):
    """Roda o estimador para todos os pares (L1[i], L2[i]) de uma vez. Devolve inovação e dθ2 (G×n)."""
    L1, L2 = np.atleast_1d(L1).astype(float), np.atleast_1d(L2).astype(float)
    L3 = np.broadcast_to(np.atleast_1d(L3).astype(float), L1.shape)   # ganho da perturbação d
    th, a, v1 = D["theta2"], D["a"], D["dtheta1"]
    if acel_real:   # aceleração realmente aplicada no intervalo (o dθ1 do firmware já tem frenagem e saturações)
        a = np.append(np.diff(v1) / TS, 0.0)   # a[k-1] = (v1[k] - v1[k-1]) / Ts
    n, g = len(th), len(L1)
    x2 = np.full(g, th[0]); w2 = np.full(g, D["dtheta2"][0]); d = np.zeros(g)   # d: θ̈2 extra (passeio aleatório)
    inov = np.zeros((g, n)); est = np.zeros((g, n)); est[:, 0] = w2
    d_at = min(max(atraso_ms * 1e-3, 0.0), TS)

    def f(x2, w2, w1, ac):
        r2 = -m["mu2"] * w2 + 0.5 * m["be"] * w1 ** 2 * np.sin(2 * x2) + m["et"] * np.sin(x2)
        return w2, (r2 - m["ga"] * np.cos(x2) * ac) / m["de"] + d

    def integra(x2, w2, w1, ac, h):
        if h <= 0:
            return x2, w2, w1
        dt = h / nsub
        for _ in range(nsub):
            k1 = f(x2, w2, w1, ac)
            k2 = f(x2 + dt / 2 * k1[0], w2 + dt / 2 * k1[1], w1 + dt / 2 * ac, ac)
            k3 = f(x2 + dt / 2 * k2[0], w2 + dt / 2 * k2[1], w1 + dt / 2 * ac, ac)
            k4 = f(x2 + dt * k3[0], w2 + dt * k3[1], w1 + dt * ac, ac)
            x2 = x2 + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
            w2 = w2 + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
            w1 = w1 + dt * ac
        return x2, w2, w1

    for k in range(1, n):
        # durante [k-1, k]: o comando a[k-2] ainda vale por atraso_ms, depois entra a[k-1]
        w1 = v1[k - 1]
        a_ant = a[k - 2] if k >= 2 else a[k - 1]
        x2, w2, w1 = integra(x2, w2, w1, a_ant, d_at)
        x2, w2, w1 = integra(x2, w2, w1, a[k - 1], TS - d_at)
        e = th[k] - (x2 - atraso_med_ms * 1e-3 * w2)   # a medida é de atraso_med_ms atrás
        inov[:, k] = e
        x2 = x2 + L1 * e
        w2 = w2 + L2 * e
        d = d + L3 * e
        est[:, k] = w2
    return inov, est


def referencia(D):
    b, a = butter(3, 6 / (0.5 / TS))
    return filtfilt(b, a, np.gradient(D["theta2"], TS)), (b, a)


def analisa(arq, m, atraso, L1g, L2g):
    D = G.le(arq); M = G.metricas(D); t = D["t"]
    cap = M["captura"] if M and M["captura"] == M["captura"] else None
    if cap is None:
        print(f"{arq}: sem trecho de equilíbrio, pulado"); return None
    eq = (np.abs(D["w"]) < 0.35) & (t > cap + 1)
    tudo = t > 0.5
    LL1, LL2 = np.meshgrid(L1g, L2g, indexing="ij")
    inov, est = filtro(D, LL1.ravel(), LL2.ravel(), m, atraso)
    ref, (b, a) = referencia(D)
    res = {}
    sw = (t > 0.5) & (t < cap)
    for nome, msk in (("equilíbrio", eq), ("swing-up", sw), ("ensaio inteiro", tudo)):
        if msk.sum() < 20: continue
        J = np.nan_to_num((inov[:, msk] ** 2).mean(axis=1), nan=np.inf)
        i = int(np.argmin(J))
        e = inov[i, msk]
        hf = est[i] - filtfilt(b, a, est[i])
        i0 = int(np.argmin(np.abs(LL1.ravel() - 0.5) + np.abs(LL2.ravel() - 20)))   # ganhos atuais
        hf0 = est[i0] - filtfilt(b, a, est[i0])
        res[nome] = dict(L1=LL1.ravel()[i], L2=LL2.ravel()[i], inov=np.sqrt(J[i]) / Q, inov0=np.sqrt(J[i0]) / Q,
                         r1=np.corrcoef(e[:-1], e[1:])[0, 1], ruido=hf[eq].std(), ruido0=hf0[eq].std(),
                         erro=(est[i] - ref)[eq].std(), erro0=(est[i0] - ref)[eq].std(),
                         borda=LL1.ravel()[i] in (L1g[0], L1g[-1]) or LL2.ravel()[i] in (L2g[0], L2g[-1]))
    return res


def main(argv):
    ini, atraso, arqs = "furuta.ini", 0.0, []
    it = iter(argv)
    for x in it:
        if x == "--ini": ini = next(it)
        elif x == "--atraso": atraso = float(next(it))
        else: arqs.append(x)
    if not arqs:
        print(__doc__); return 2
    m = le_fisico(ini)
    L1g = np.array([0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0])
    L2g = np.array([0.5, 1, 2, 3, 5, 7, 10, 14, 20, 28, 40, 60, 90, 130, 200])
    print(f"atraso do comando na predição: {atraso} ms | atual: L1 0.5, L2 20\n")
    print("ensaio   trecho          |  L1    L2  | inovação RMS (LSB) ótimo/atual | autocorr. inov. | ruído dθ2 ótimo/atual | erro dθ2 vs ref ótimo/atual")
    for arq in arqs:
        r = analisa(arq, m, atraso, L1g, L2g)
        if not r: continue
        for nome, x in r.items():
            print(f"{arq[-10:-4]}   {nome:15s} | {x['L1']:4.2f} {x['L2']:5.1f}{'*' if x['borda'] else ' '}| "
                  f"{x['inov']:5.2f} / {x['inov0']:5.2f}                  | {x['r1']:+.2f}           | "
                  f"{x['ruido']:.3f} / {x['ruido0']:.3f}         | {x['erro']:.3f} / {x['erro0']:.3f}")
    print("\n* = ótimo na borda da grade")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
