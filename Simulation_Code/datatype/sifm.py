from dataclasses import dataclass


@dataclass
class SIFM:
    # default parameters for Social Interaction Field Model
    # (original paper: https://doi.org/10.1038/s41562-019-0618-2)
    m1: float = 0.438
    n1: float = 0.63
    m2: float = 0.321
    n2: float = 0.856
    a: float = 0.24
    b: float = 0.12
    c: float = 1.43
    th: float = 10.18
    ncf: float = 2.0
