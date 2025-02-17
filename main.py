import math
radio = float(input("Introduce el radio de la circunferencia: "))
perimetro = 2 * math.pi * radio
area = math.pi * radio ** 2
print(f"El perímetro de la circunferencia es {perimetro:.2f} y el área es {area:.2f}")