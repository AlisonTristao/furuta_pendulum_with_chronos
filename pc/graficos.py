"""Gráfico de um ensaio do malha.exe e tabela de métricas dos ensaios de swing-up.

    python graficos.py ensaios/ensaio_X.csv [saida.png]
    python graficos.py --tabela rotulos.csv [saida.md]     (rotulos.csv: arquivo,config)

Precisa de numpy e matplotlib (ex.: Python do Anaconda). Ângulos: θ2 = 0 no topo.
"""
import csv
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import numpy as np

TOPO = 0.35        # |θ2| (rad) considerado "no topo"
CAPTURA = 0.3      # |θ2| da primeira subida
COR = "#2a78d6"    # série única por painel
TEXTO, TEXTO2, GRADE, FUNDO = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def le(arq):
    with open(arq) as f:
        r = list(csv.DictReader(f))
    D = {k: np.array([float(x[k]) for x in r]) for k in r[0]}
    th = D["theta2"]
    D["w"] = th - 2 * np.pi * np.floor((th + np.pi) / (2 * np.pi))   # (-π, π]
    return D


def trechos(t, mask):
    """[(i0, i1)] dos trechos contíguos com mask verdadeiro."""
    out, s = [], None
    for i, m in enumerate(mask):
        if m and s is None:
            s = i
        if s is not None and (not m or i == len(mask) - 1):
            out.append((s, i))
            s = None
    return out


def metricas(D):
    t, w = D["t"], D["w"]
    up = np.abs(w) < TOPO
    i_sub = next((i for i in range(len(t)) if abs(w[i]) < CAPTURA), None)
    if i_sub is None:
        return None
    t_sub = t[i_sub]
    tr = trechos(t, up)
    maior = max((t[b] - t[a] for a, b in tr), default=0.0)
    quedas = sum(1 for a, b in tr if b < len(t) - 1 and t[b] > t_sub)
    pos = t >= t_sub
    m = up & (t > t_sub + 1.0)                     # equilíbrio (sem o primeiro segundo da captura)
    cap = next((t[a] for a, b in tr if t[b] - t[a] >= 1.0), np.nan)   # 1º trecho de >= 1 s no topo
    return dict(
        subida=t_sub,
        captura=cap,
        topo=100 * up[pos].mean(),
        maior=maior,
        quedas=quedas,
        rms_t2=np.degrees(np.sqrt((w[m] ** 2).mean())) if m.any() else np.nan,
        rms_a=np.sqrt((D["a"][m] ** 2).mean()) if m.any() else np.nan,
        max_t1=np.abs(D["theta1"]).max(),
        osc_t1=D["theta1"][m].std() if m.any() else np.nan,   # oscilação do braço no equilíbrio
        dur=t[-1],
    )


def grafico(arq, saida):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    D = le(arq)
    t = D["t"]
    M = metricas(D)
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRADE, "axes.labelcolor": TEXTO2,
                         "xtick.color": TEXTO2, "ytick.color": TEXTO2, "text.color": TEXTO})
    fig, ax = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True, facecolor=FUNDO)
    w = np.degrees(D["theta2"])                      # contínuo (sem dobrar em ±180°): topo em 0 ou ±360
    paineis = [
        (w, "Ângulo do pêndulo θ2 (graus; 0 e ±360 = topo, ±180 = embaixo)"),
        (D["theta1"], "Ângulo do braço θ1 (rad)"),
        (D["a"], "Aceleração comandada do braço (rad/s²)"),
    ]
    for a, (y, titulo) in zip(ax, paineis):
        a.set_facecolor(FUNDO)
        a.plot(t, y, color=COR, lw=1.2)
        a.set_title(titulo, loc="left", fontsize=10, color=TEXTO)
        a.grid(axis="y", color=GRADE, lw=0.8)
        a.axhline(0, color=TEXTO2, lw=0.6)
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
        if M:
            a.axvline(M["subida"], color=TEXTO2, lw=0.8, ls="--")
    ax[0].set_ylim(-360, 360)
    ax[0].set_yticks(np.arange(-360, 361, 90))
    for v in (1.46, -1.46):
        ax[1].axhline(v, color=TEXTO2, lw=0.8, ls=":")
    ax[1].text(t[-1], 1.46, "limite de θ1 do MPC ", va="bottom", ha="right", fontsize=8, color=TEXTO2)
    if M:
        ax[0].annotate(f"subida em {M['subida']:.2f} s", (M["subida"], 120), xytext=(6, 0),
                       textcoords="offset points", fontsize=8, color=TEXTO2)
    ax[2].set_xlabel("tempo (s)")
    nome = arq.replace("\\", "/").split("/")[-1]
    if M:
        sub = (f"{nome}  ·  topo {M['topo']:.0f}% do tempo após a subida  ·  maior trecho {M['maior']:.1f} s  ·  "
               f"{M['quedas']} quedas  ·  RMS de θ2 no equilíbrio {M['rms_t2']:.2f}°")
    else:
        sub = nome
    fig.suptitle("Swing-up e equilíbrio do pêndulo de Furuta (NMPC qLPV, CHRONOS)", x=0.06, ha="left",
                 fontsize=12, color=TEXTO)
    fig.text(0.06, 0.935, sub, fontsize=8.5, color=TEXTO2)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(saida, dpi=150, facecolor=FUNDO)
    print("gráfico:", saida)


def tabela(rotulos, saida):
    with open(rotulos, encoding="utf-8") as f:
        linhas = [r for r in csv.reader(f) if r and not r[0].startswith("#")]
    cab = ("| Ensaio | Configuração | Subida (s) | Topo após a subida | Maior trecho no topo (s) | Quedas | "
           "RMS θ2 no equilíbrio (°) | RMS θ̈1 (rad/s²) | Máx \\|θ1\\| (rad) |")
    out = [cab, "|" + "---|" * 9]
    for arq, cfg in linhas:
        M = metricas(le(arq))
        nome = arq.replace("\\", "/").split("/")[-1].replace("ensaio_", "").replace(".csv", "")
        hora = f"{nome[9:11]}:{nome[11:13]}"
        if M is None:
            out.append(f"| {hora} | {cfg} | não subiu | – | – | – | – | – | – |")
            continue
        out.append(f"| {hora} | {cfg} | {M['subida']:.2f} | {M['topo']:.0f}% | {M['maior']:.1f} | {M['quedas']} | "
                   f"{M['rms_t2']:.2f} | {M['rms_a']:.0f} | {M['max_t1']:.2f} |")
    txt = "\n".join(out) + "\n"
    print(txt)
    if saida:
        with open(saida, "w", encoding="utf-8") as f:
            f.write(txt)
        print("tabela:", saida)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    if sys.argv[1] == "--tabela":
        tabela(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None)
    else:
        grafico(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else sys.argv[1].replace(".csv", ".png"))
