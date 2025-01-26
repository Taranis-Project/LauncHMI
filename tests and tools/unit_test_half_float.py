import struct

def half_float_from_hex(hex_values):
    """
    Convert a half-precision float (16-bit) represented by 2 hexadecimal values
    into a single-precision float (32-bit).

    Args:
        hex_values: A list of 2 hexadecimal values (as strings or bytes) that represent a half-precision float.

    Returns:
        A 32-bit float converted from the hexadecimal values.
    """
    # Si hex_values sont des chaînes hexadécimales, les convertir en bytes
    if isinstance(hex_values[0], str):
        hex_values = [bytes.fromhex(value[2:]) for value in hex_values]

    # Vérification que la liste contient exactement 2 octets (pour un half float)
    if len(hex_values) != 2:
        raise ValueError("Il faut exactement 2 octets pour un half float 16 bits.")

    # Inverser les octets pour l'endianness si nécessaire (big-endian vers little-endian)
    hex_values.reverse()

    # Concaténer les octets
    byte_data = b''.join(hex_values)

    # Unpacker les 2 octets en un half-float 16 bits
    half_float_bits = struct.unpack('<H', byte_data)[0]  # Little-endian unpack pour un half float

    # Extraire les parties du half-float (sign, exposant, mantisse)
    sign = (half_float_bits >> 15) & 0x1
    exponent = (half_float_bits >> 10) & 0x1F
    mantissa = half_float_bits & 0x3FF

    # Calcul du nombre réel en format IEEE 754 demi-précision
    if exponent == 0:
        # Cas des sous-normaux
        float_value = mantissa * (2 ** -24)
    elif exponent == 31:
        # Cas de l'infini ou NaN
        if mantissa == 0:
            float_value = float('inf') if sign == 0 else float('-inf')
        else:
            float_value = float('nan')
    else:
        # Cas des nombres normaux
        float_value = (1 + mantissa * (2 ** -10)) * (2 ** (exponent - 15))

    # Appliquer le signe
    if sign == 1:
        float_value = -float_value

    return float_value

# Exemple avec un half float
hex_values = [b'\x3c', b'\x00']  # Représentation de 1.0 en half float
result = half_float_from_hex(hex_values)
print(result)  # Cela devrait imprimer 1.0