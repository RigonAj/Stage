import numpy as np
import matplotlib.pyplot as plt

x = np.array([422,444,442,421,423,411,451,448,430,449,413,409,424,444,412,435,420,437,430,422,436,445,437,434,444,433,443,414,444,407,433,434,444,438,443,430,430,425,437,429,434,420,419,421,417])
y = np.array([362,358,325,362,363,353,343,332,320,331,357,351,364,327,357,362,359,323,364,363,341,327,322,320,326,320,325,358,325,343,364,365,359,363,359,357,319,362,363,319,362,361,361,362,361])

A = np.column_stack([2*x, 2*y, np.ones_like(x)])
b = x**2 + y**2
sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
xc, yc, c = sol
r = np.sqrt(xc**2 + yc**2 + c)
print(xc,yc,r)




import re

import re

def format_arrays(input_str):
    return re.sub(r'(?<=\d)\s+(?=\d)', ',', input_str)


# Exemple d'utilisation
s = """[422 444 442 421 423 411 451 448 430 449 413 409 424 444 412 435 420 437
 430 422 436 445 437 434 444 433 443 414 444 407 433 434 444 438 443 430
 430 425 437 429 434 420 419 421 417] [362 358 325 362 363 353 343 332 320 331 357 351 364 327 357 362 359 323
 364 363 341 327 322 320 326 320 325 358 325 343 364 365 359 363 359 357
 319 362 363 319 362 361 361 362 361]"""

result = format_arrays(s)
print(result)


# Graphique
plt.figure(figsize=(8,5))
plt.scatter(x, y)
plt.xlabel("x")
plt.ylabel("y")
plt.title("Nuage de points")
plt.grid(True)
plt.show()
