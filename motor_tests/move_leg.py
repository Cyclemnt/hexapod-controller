import time
from scservo_sdk import PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS

DEVICENAME = '/dev/ttyAMA0'
BAUDRATE = 1000000
LEG_IDS = [0, 1, 2]  # 0: Coxa, 1: Fémur, 2: Tibia

# Registres mémoire ST3215
ADDR_TORQUE_ENABLE = 40
ADDR_SYNC_START    = 41  # Début du bloc contigu (Accélération + Position + Temps + Vitesse)
SYNC_DATA_LENGTH   = 7   # 1 + 2 + 2 + 2 octets

# Butées logicielles de sécurité (à ajuster selon les débattements de votre CAO)
MIN_POS = 1200
MAX_POS = 2900

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(0)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_SYNC_START, SYNC_DATA_LENGTH)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("Erreur : impossible d'ouvrir /dev/ttyAMA0")
    quit()

def clamp(val, min_v, max_v):
    return max(min_v, min(max_v, val))

def move_leg_sync(positions, speed=800, accel=40):
    """Envoie simultanément les consignes [pos0, pos1, pos2] aux 3 moteurs."""
    groupSyncWrite.clearParam()

    for servo_id, raw_pos in zip(LEG_IDS, positions):
        pos = clamp(int(raw_pos), MIN_POS, MAX_POS)

        # Construction du tableau de 7 octets (Little-Endian pour les valeurs 16 bits)
        param = [
            accel & 0xFF,
            pos & 0xFF, (pos >> 8) & 0xFF,
            0, 0,  # Goal Time = 0 (priorité à la vitesse)
            speed & 0xFF, (speed >> 8) & 0xFF
        ]

        if not groupSyncWrite.addParam(servo_id, param):
            print(f"Erreur addParam sur le moteur ID {servo_id}")
            return False

    comm_result = groupSyncWrite.txPacket()
    if comm_result != COMM_SUCCESS:
        print(f"Erreur SyncWrite : {packetHandler.getTxRxResult(comm_result)}")
        return False
    return True

# 1. Vérification de la présence des 3 moteurs et activation du couple
for sid in LEG_IDS:
    _, res, _ = packetHandler.ping(portHandler, sid)
    if res != COMM_SUCCESS:
        print(f"Moteur ID {sid} introuvable sur le bus. Vérifiez le chaînage.")
        portHandler.closePort()
        quit()
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE_ENABLE, 1)

print("Patte connectée (IDs 0, 1, 2). Mise en position neutre [2048, 2048, 2048]...")
move_leg_sync([2048, 2048, 2048], speed=600, accel=30)
time.sleep(1.5)

# 2. Mode interactif pour tester des postures
print("\nEntrez 3 positions séparées par des espaces (ex: 2200 1800 2300), ou 'q' pour quitter :")
try:
    while True:
        cmd = input("Cible [Coxa Fémur Tibia] > ").strip()
        if cmd.lower() == 'q':
            break

        parts = cmd.split()
        if len(parts) != 3 or not all(p.isdigit() for p in parts):
            print("Format invalide. Saisissez 3 entiers (ex: 2048 1900 2200).")
            continue

        targets = [int(p) for p in parts]
        move_leg_sync(targets, speed=800, accel=40)

finally:
    # Retour en douceur au neutre avant de couper le script (le couple reste actif)
    print("Retour au neutre [2048, 2048, 2048]...")
    move_leg_sync([2048, 2048, 2048], speed=600, accel=30)
    time.sleep(1.0)
    portHandler.closePort()