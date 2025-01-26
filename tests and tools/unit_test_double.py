import struct
import os

def process_double(byte_data):
    try:
        # Vérifier si byte_data contient au moins size_double octets
        if len(byte_data) < 8:
            print("Erreur: Pas assez d'octets pour un double.")
            return

        # Extraire et imprimer les octets pour le double
        byte_data.reverse()
        print(byte_data[:8])

        # Convertir les octets en double (64 bits)
        data = struct.unpack('d', b''.join(byte_data[:8]))[0]

        # Afficher la valeur du double
        print("double:")
        print(data)

        # Pause pour inspection (Windows)
        os.system("pause")

    except Exception as e:
        print(f"Erreur lors de la conversion du double: {e}")

# Exemple de byte_data contenant des octets pour un double (64 bits)
byte_data = [b'\x40', b'\x59', b'\x0f', b'\xd3', b'\x8f', b'\x5c', b'\x2f', b'\x40']

# Appeler la fonction pour traiter le double
process_double(byte_data)