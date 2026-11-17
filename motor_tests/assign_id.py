import time
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DEVICENAME = '/dev/ttyAMA0'
BAUDRATE = 1000000

# Registres mémoire Feetech ST3215 (Protocole 0)
ADDR_ID            = 5    # 1 octet  (EEPROM)
ADDR_TORQUE_ENABLE = 40   # 1 octet  (SRAM : 0 = libre, 1 = actif)
ADDR_ACCELERATION  = 41   # 1 octet  (SRAM)
ADDR_GOAL_POSITION = 42   # 2 octets (SRAM : 0 à 4095, centre = 2048)
ADDR_GOAL_SPEED    = 46   # 2 octets (SRAM)
ADDR_LOCK          = 55   # 1 octet  (SRAM : 0 = EEPROM déverrouillée, 1 = verrouillée)

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(0)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("Erreur : impossible d'ouvrir /dev/ttyAMA0 à 1 000 000 bauds.")
    quit()

def find_single_servo():
    found_ids = []
    for candidate_id in range(0, 20):
        _, comm_result, _ = packetHandler.ping(portHandler, candidate_id)
        if comm_result == COMM_SUCCESS:
            found_ids.append(candidate_id)
    return found_ids

print("=== Assistant de configuration des 18 ST3215 (IDs 0 à 17) ===")
print("Tapez 'q' à tout moment pour quitter.\n")

try:
    while True:
        user_input = input("1. Branchez UN SEUL moteur, puis entrez son nouvel ID (0 à 17) : ").strip()
        if user_input.lower() == 'q':
            break

        if not user_input.isdigit() or not (0 <= int(user_input) <= 17):
            print("Entrée invalide. Saisissez un nombre entier entre 0 et 17.\n")
            continue

        new_id = int(user_input)
        detected = find_single_servo()

        if len(detected) == 0:
            print("Aucun moteur détecté (IDs 0 à 19). Vérifiez la LiPo et le fil Signal.\n")
            continue
        if len(detected) > 1:
            print(f"ALERTE : Plusieurs moteurs détectés ({detected}). Débranchez-en un avant de continuer.\n")
            continue

        current_id = detected[0]
        print(f"-> Moteur détecté sur l'ID actuel : {current_id}")

        if current_id != new_id:
            # Désactiver le couple et déverrouiller l'EEPROM
            packetHandler.write1ByteTxRx(portHandler, current_id, ADDR_TORQUE_ENABLE, 0)
            packetHandler.write1ByteTxRx(portHandler, current_id, ADDR_LOCK, 0)
            time.sleep(0.05)

            # Écrire le nouvel ID
            comm_result, _ = packetHandler.write1ByteTxRx(portHandler, current_id, ADDR_ID, new_id)
            time.sleep(0.05)

            # Reverrouiller l'EEPROM sur le nouvel ID
            packetHandler.write1ByteTxRx(portHandler, new_id, ADDR_LOCK, 1)
            time.sleep(0.05)

            if comm_result != COMM_SUCCESS:
                print(f"Échec d'écriture de l'ID : {packetHandler.getTxRxResult(comm_result)}\n")
                continue

        # Vérification par Ping et alignement au zéro mécanique (2048)
        _, comm_result, _ = packetHandler.ping(portHandler, new_id)
        if comm_result == COMM_SUCCESS:
            packetHandler.write1ByteTxRx(portHandler, new_id, ADDR_TORQUE_ENABLE, 1)
            packetHandler.write1ByteTxRx(portHandler, new_id, ADDR_ACCELERATION, 50)
            packetHandler.write2ByteTxRx(portHandler, new_id, ADDR_GOAL_SPEED, 1000)
            packetHandler.write2ByteTxRx(portHandler, new_id, ADDR_GOAL_POSITION, 2048)
            time.sleep(0.8)
            print(f"SUCCÈS : Moteur configuré sur ID {new_id} et maintenu à 2048.")
            print(f"-> Étiquetez-le 'ID {new_id}', débranchez-le, puis passez au suivant.\n")
        else:
            print(f"Erreur : le moteur ne répond pas sur l'ID {new_id} après écriture.\n")

finally:
    portHandler.closePort()
    print("Port série fermé proprement.")