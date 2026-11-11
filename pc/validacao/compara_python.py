"""Validação offline do controlador em C contra o de referência do qlpv_nn_furuta (python/nmpc, config 'icit_chronos'
= ZOH + ρ médio + Picard até 5 QPs + QP do CHRONOS com folgas recalculadas + terminal DLQR).

Dá ao controlador Python a mesma sequência de estados e referências do CSV do sim.exe (cada um com seu warm start)
e compara o u passo a passo. Só para o pêndulo do artigo (artigo.ini).

    python validacao/compara_python.py sim_artigo.csv [--orig C:/Users/.../qlpv_nn_furuta/python]
"""
import argparse
import os
import sys

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--orig', default=os.path.expanduser('~/git/qlpv_nn_furuta/python'))
    a = ap.parse_args()
    sys.path.insert(0, a.orig)
    from nmpc import modelo
    from nmpc.controlador import Controller
    from nmpc.configuracoes import opcoes

    d = np.genfromtxt(a.csv, delimiter=',', names=True)
    X = np.c_[d['theta1'], d['theta2'], d['dtheta1'], d['dtheta2']]
    p = modelo.params(Ts=0.02, vmax=30.0, disc='zoh')
    c = Controller(p, X[0], 0.0, **opcoes('icit_chronos'))
    du, dq = [], 0
    for k in range(len(X)):
        ref = np.zeros((4, c.N))
        ref[0] = d['theta1_ref'][k]
        u, info = c.step(X[k], ref)
        du.append(abs(u - d['u'][k]))
        dq += info['n_qp'] != d['n_qp'][k]
    du = np.array(du)
    print(f'{len(du)} passos: max|u_C - u_py| = {du.max():.2e} (mediana {np.median(du):.1e}, '
          f'passo do máximo {du.argmax()}), passos com n_qp diferente: {dq}')
    for lim in (1e-9, 1e-6, 1e-3):
        print(f'  passos com |du| > {lim:g}: {np.sum(du > lim)}')


if __name__ == '__main__':
    main()
