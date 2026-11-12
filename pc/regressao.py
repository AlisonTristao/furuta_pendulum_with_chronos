"""Regressão dos parâmetros do pêndulo com ensaios de swing-up/equilíbrio (mínimos quadrados, linear nos parâmetros).

    python regressao.py ensaios/ensaio_A.csv [...] [--ini furuta.ini]

A equação do pêndulo (dividida por J = I2 + m2 l2², por massa) é linear nos coeficientes:

    θ̈2 = c_g sin θ2 + c_c dθ1² sin 2θ2 − c_a cos θ2 θ̈1 − c_v dθ2 − c_s sinal(dθ2)

    c_g = g l2/J (= ωn²),  c_c = l2²/(2J),  c_a = L1 l2/J,  c_v = μ2/J,  c_s = Tc/J

θ̈1 é a aceleração realmente aplicada pelo firmware (diferença do dθ1, que é a velocidade comandada dos passos).
Ela entra com atrasos de -1 a +2 amostras em regressores separados: o modelo espera tudo no atraso 0. θ̈2 e dθ2
vêm de θ2 suavizado (Savitzky-Golay). Dos coeficientes: l2 = 2 c_c g / c_g e L1 efetivo = c_a g / c_g (o L1
medido é 9,145 cm; L1 efetivo diferente = o braço não acelera o que o firmware acha: passo perdido, folga, flexão).
"""
import configparser
import sys

import numpy as np
from scipy.signal import savgol_filter

import graficos as G

TS, g = 0.02, 9.81


def le_fisico(ini):
    cp = configparser.ConfigParser(inline_comment_prefixes=(";",), strict=False)
    cp.optionxform = str   # chaves com maiúscula (I2, L1)
    cp.read(ini, encoding="utf-8")
    return {k: float(v) for k, v in cp["fisico"].items()}


def regressores(D, atrasos=(-1, 0, 1, 2), so_balanco=False):
    th = savgol_filter(D["theta2"], 7, 3)
    w2 = savgol_filter(D["theta2"], 7, 3, deriv=1, delta=TS)
    al2 = savgol_filter(D["theta2"], 7, 3, deriv=2, delta=TS)
    v1 = D["dtheta1"]
    a = np.append(np.diff(v1) / TS, 0.0)            # a[k]: aceleração aplicada em [k, k+1]
    am = np.convolve(a, [0.5, 0.5])[: len(a)]       # média em [k-1, k+1] (θ̈2 central em k)
    k = np.arange(5, len(th) - 5)
    cols = [np.sin(th[k]), v1[k] ** 2 * np.sin(2 * th[k])]
    nomes = ["c_g", "c_c"]
    for d in atrasos:
        cols.append(-np.cos(th[k]) * am[k - d])
        nomes.append("c_a[%+d]" % d)
    cols += [-w2[k], -np.sign(w2[k])]
    nomes += ["c_v", "c_s"]
    X = np.column_stack(cols)
    y = al2[k]
    m = np.ones(len(k), bool)
    if so_balanco:                                   # tira os trechos parados (só ruído)
        m = (np.abs(w2[k]) > 0.3) | (np.abs(a[k]) > 5)
    return X[m], y[m], nomes


def ajusta(X, y):
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ c
    return c, 1 - r.var() / y.var(), r.std()


def main(argv):
    ini, arqs = "furuta.ini", []
    it = iter(argv)
    for x in it:
        if x == "--ini": ini = next(it)
        else: arqs.append(x)
    if not arqs:
        print(__doc__); return 2
    f = le_fisico(ini)
    J = f["I2"] / f["m2"] + f["l2"] ** 2
    mod = dict(c_g=g * f["l2"] / J, c_c=f["l2"] ** 2 / (2 * J), c_a=f["L1"] * f["l2"] / J, c_v=f["mu2"] / f["m2"] / J)
    print("modelo do %s: c_g %.1f  c_c %.3f  c_a %.3f  c_v %.3f  (l2 %.4f, L1 %.4f, período %.3f s)\n" % (
        ini, mod["c_g"], mod["c_c"], mod["c_a"], mod["c_v"], f["l2"], f["L1"], 2 * np.pi / np.sqrt(mod["c_g"])))
    print("ensaio  | R²    | c_g    c_c    c_a[-1] c_a[0] c_a[+1] c_a[+2] c_v    c_s   | período l2 ef.  L1 ef. (soma c_a) | atraso da aceleração")
    todos = []
    for arq in arqs:
        D = G.le(arq)
        X, y, nomes = regressores(D, so_balanco=True)
        todos.append((X, y))
        c, r2, _ = ajusta(X, y)
        _relata(arq[-10:-4], c, r2)
    X = np.vstack([t[0] for t in todos]); y = np.concatenate([t[1] for t in todos])
    c, r2, _ = ajusta(X, y)
    _relata("TODOS ", c, r2)
    return 0


def _relata(nome, c, r2):
    cg, cc = c[0], c[1]
    ca = c[2:6]
    cv, cs = c[6], c[7]
    soma = ca.sum()
    atraso = (ca * np.array([-1, 0, 1, 2])).sum() / soma * TS * 1e3 if soma else np.nan
    per = 2 * np.pi / np.sqrt(cg) if cg > 0 else np.nan
    print("%s  | %.3f | %6.1f %6.3f %7.3f %6.3f %7.3f %7.3f %6.3f %6.2f | %.3f s %.4f  %.4f          | %+.1f ms" % (
        nome, r2, cg, cc, ca[0], ca[1], ca[2], ca[3], cv, cs, per, 2 * cc * g / cg, soma * g / cg, atraso))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
