import sys, numpy as np
sys.path.insert(0, r"c:\git\furuta_pendulum_with_chronos\pc")
import graficos as G
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
D = sys.argv[1]
casos = [("base", "Atual (peso uniforme, θ2 contínuo)"), ("a3", "Teste 1: peso exponencial α = 3"),
         ("wrap", "Teste 2: θ2 dobrado em ±π"), ("wrapa2", "Testes 1 + 2: dobrado + α = 2")]
cores = ["#2a78d6", "#e08a1e", "#7a4fc9"]   # uma cor por semente (ensaio), mesma ordem em todos os painéis
plt.rcParams.update({"font.size": 9, "axes.edgecolor": G.GRADE, "xtick.color": G.TEXTO2, "ytick.color": G.TEXTO2})
fig, ax = plt.subplots(4, 2, figsize=(13, 10), sharex=True, facecolor=G.FUNDO, gridspec_kw={"width_ratios": [1.4, 1]})
for i, (k, tit) in enumerate(casos):
    subs = []
    for s, cor in zip((1, 2, 3), cores):
        X = G.le(f"{D}/{k}_s{s}.csv"); t = X["t"]; m = t <= 8; M = G.metricas(X)
        subs.append(f"{M['captura']:.1f} s" if M and M['captura'] == M['captura'] else "não")
        ax[i, 0].plot(t[m], np.degrees(X["theta2"][m]), color=cor, lw=1.1, label=f"ensaio {s}")
        ax[i, 1].plot(t[m], X["theta1"][m], color=cor, lw=1.0)
    ax[i, 0].set_title(f"{tit}  ·  captura em {', '.join(subs)}", loc="left", fontsize=10)
    ax[i, 0].set_ylim(-400, 400); ax[i, 0].set_yticks(np.arange(-360, 361, 180)); ax[i, 0].set_ylabel("θ2 (graus)", color=G.TEXTO2)
    ax[i, 1].set_ylim(-1.7, 1.7); ax[i, 1].set_ylabel("θ1 (rad)", color=G.TEXTO2)
    for a in ax[i]:
        a.set_facecolor(G.FUNDO); a.grid(axis="y", color=G.GRADE, lw=0.7); a.axhline(0, color=G.TEXTO2, lw=0.5)
        for sp in ("top", "right"): a.spines[sp].set_visible(False)
ax[0, 0].legend(loc="upper right", fontsize=8, frameon=False, ncol=3)
for a in ax[3]: a.set_xlabel("tempo (s)")
fig.suptitle("Swing-up simulado: 3 ensaios (sementes) por opção  ·  θ2: 0 e ±360 = topo, ±180 = embaixo", x=0.05, ha="left", fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.965)); fig.savefig(sys.argv[2], dpi=125, facecolor=G.FUNDO)
