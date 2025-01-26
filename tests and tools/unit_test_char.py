# Exemple de byte_data contenant un seul byte
byte_data = [b'\x41']  # Liste contenant un byte (représente 'A' en ASCII)

# Extraire l'entier à partir du byte
data = byte_data[0][0]  # 0x41 -> 65 (l'entier)

# Utiliser chr pour obtenir le caractère correspondant
char = chr(data)  # Convertir 65 en 'A'

print(f"Valeur numérique: {data}")  # Affiche 65
print(f"Caractère: {char}")  # Affiche 'A'