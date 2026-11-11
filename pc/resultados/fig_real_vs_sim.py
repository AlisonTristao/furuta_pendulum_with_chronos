import sys, numpy as np
sys.path.insert(0, r"c:\git\furuta_pendulum_with_chronos\pc")
import graficos as G
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
D = sys.argv[1]
casos = [("Planta real (ensaio 23:35)", "ensaios/ensaio_20261110_233534.csv", "#0b0b0b"),
         ("Simulação nova (atraso 3,5 ms, atrito real, motor e encoder do firmware)", D + "/nova_6.csv", G.COR),
         ("Simulação antiga (atraso 20 ms, atrito do modelo)", D + "/antiga.csv", "#c0392b")]
plt.rcParams.update({"font.size": 9, "axes.edgecolor": G.GRADE, "xtick.color": G.TEXTO2, "ytick.color": G.TEXTO2})
fig, ax = plt.subplots(3, 2, figsize=(13, 8.5), sharex=True, facecolor=G.FUNDO, gridspec_kw={"width_ratios": [1.3, 1]})
for i, (tit, arq, cor) in enumerate(casos):
    X = G.le(arq); M = G.metricas(X); t = X["t"]
    for j, (y, lab) in enumerate(((np.degrees(X["theta2"]), "θ2 (graus)"), (X["a"], "aceleração comandada (rad/s²)"))):
        a = ax[i, j]; a.set_facecolor(G.FUNDO); a.plot(t, y, color=cor, lw=1.0)
        a.axhline(0, color=G.TEXTO2, lw=0.5); a.grid(axis="y", color=G.GRADE, lw=0.7)
        for s in ("top", "right"): a.spines[s].set_visible(False)
        a.set_ylabel(lab, color=G.TEXTO2)
        if j == 0: a.set_ylim(-360, 360); a.set_yticks(np.arange(-360, 361, 180))
        else: a.set_ylim(-170, 170)
    met = (f"topo {M['topo']:.0f}% · {M['quedas']} quedas · RMS θ2 {M['rms_t2']:.1f}° · RMS acel. {M['rms_a']:.0f}"
           if M else "não subiu")
    ax[i, 0].set_title(f"{tit}\n{met}", loc="left", fontsize=9.5, color=G.TEXTO)
for a in ax[2]: a.set_xlabel("tempo (s)")
fig.suptitle("Mesma configuração (u ±0,75, estimador, compensação 20 ms): real × simulação", x=0.05, ha="left", fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.96)); fig.savefig(sys.argv[2], dpi=130, facecolor=G.FUNDO)
