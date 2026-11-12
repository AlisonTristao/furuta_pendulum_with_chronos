import sys, numpy as np
sys.path.insert(0, r"c:\git\furuta_pendulum_with_chronos\pc")
import graficos as G
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
D = sys.argv[1]
plt.rcParams.update({"font.size": 9, "axes.edgecolor": G.GRADE, "xtick.color": G.TEXTO2, "ytick.color": G.TEXTO2})
fig, ax = plt.subplots(6, 2, figsize=(13, 13), sharex=True, facecolor=G.FUNDO, gridspec_kw={"width_ratios": [1.4, 1]})
for i in range(6):
    X = G.le(f"{D}/c_s{i+1}.csv"); t = X["t"]; M = G.metricas(X)
    cap = f"captura em {M['captura']:.1f} s" if M and M["captura"] == M["captura"] else "não capturou em 12 s"
    ax[i, 0].plot(t, np.degrees(X["theta2"]), color=G.COR, lw=1.1)
    ax[i, 1].plot(t, X["theta1"], color=G.COR, lw=1.0)
    if M and M["captura"] == M["captura"]:
        for a in ax[i]: a.axvline(M["captura"], color=G.TEXTO2, lw=0.8, ls="--")
    ax[i, 0].set_title(f"Ensaio {i+1}  ·  {cap}", loc="left", fontsize=10)
    lim = 360 * np.ceil((np.abs(np.degrees(X["theta2"])).max() + 1) / 360); ax[i, 0].set_ylim(-lim - 40, lim + 40); ax[i, 0].set_yticks(np.arange(-lim, lim + 1, 360 if lim > 360 else 180)); ax[i, 0].set_ylabel("θ2 (graus)", color=G.TEXTO2)
    ax[i, 1].set_ylim(-1.7, 1.7); ax[i, 1].set_ylabel("θ1 (rad)", color=G.TEXTO2)
    for a in ax[i]:
        a.set_facecolor(G.FUNDO); a.grid(axis="y", color=G.GRADE, lw=0.7); a.axhline(0, color=G.TEXTO2, lw=0.5)
        for sp in ("top", "right"): a.spines[sp].set_visible(False)
for a in ax[5]: a.set_xlabel("tempo (s)")
fig.suptitle("Swing-up simulado: α = 3 só em θ2, dθ1, dθ2 (θ1 constante) + θ2 dobrado em ±π  ·  0 e ±360 = topo, ±180 = embaixo",
             x=0.04, ha="left", fontsize=11.5)
fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(sys.argv[2], dpi=115, facecolor=G.FUNDO)
