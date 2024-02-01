import sympy as sm
import sympy.physics.control as cn


def routh_table(polynomial, var):
    p = sm.poly(polynomial, var)
    n = p.degree()
    coeffs = p.coeffs()
    table = sm.zeros(n + 1, n + 1)
    first = True
    row1, row2 = [], []
    for c in coeffs:
        if first:
            row1.append(c)
            first = False
        elif not first:
            row2.append(c)
            first = True
    for i, v in enumerate(row1):
        table[0, i] = v
    for i, v in enumerate(row2):
        table[1, i] = v
    for j in range(2, n + 1):
        for i in range(n):
            table[j, i] = (table[j - 1, 0]*table[j - 2, i + 1] -
                           table[j - 2, 0]*table[j - 1, i + 1])/table[j - 1, 0]
            table[j, i] = sm.simplify(table[j, i])
    return table


Ib, Is, c, m, h, g, l2, l1 = sm.symbols('I_b, I_s, c, m, h, g, l2, l1',
                                        real=True, nonnegative=True)
KD, v = sm.symbols('K_D, v', real=True)
s = sm.symbols('s')

tau1_sq = (Ib + m*h**2)/m/g/h
tau2 = l2/v
tau3 = v/(l1 + l2)
K = v**2/g/(l1 + l2)

Gtheta = cn.TransferFunction(-K*(tau2*s + 1), tau1_sq*s**2 - 1, s)
Gdelta = cn.TransferFunction(1, Is*s**2 + c*s, s)
Gpsi = cn.TransferFunction(1, tau3*s, s)

# NOTE : Can't multiply a symbol times a TransferFunction, so awkward how to
# multiple a gain.
Ginner = cn.Feedback(cn.TransferFunction(KD*s*Gdelta.num, Gdelta.den, s), Gtheta)
char_eq = sm.simplify(Ginner.doit()).expand().den

tab = routh_table(char_eq, s)

sm.pprint(tab[:, 0])

