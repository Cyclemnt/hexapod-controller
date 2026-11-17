import time
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DEVICENAME = '/dev/ttyAMA0'
BAUDRATE = 1000000
SERVO_ID = 1

# Adresses mémoire du Feetech ST3215 (Protocole 0)
ADDR_TORQUE_ENABLE = 40      # 1 octet  (1 = actif, 0 = libre)
ADDR_ACCELERATION  = 41      # 1 octet  (0 à 254)
ADDR_GOAL_POSITION = 42      # 2 octets (0 à 4095)
ADDR_GOAL_SPEED    = 46      # 2 octets (pas/seconde)

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(0)

if not portHandler.openPort():
    print("Erreur : impossible d'ouvrir /dev/ttyAMA0")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print("Erreur : impossible de régler le baudrate")
    quit()

# 1. Ping du servomoteur
model_number, comm_result, error = packetHandler.ping(portHandler, SERVO_ID)
if comm_result != COMM_SUCCESS:
    print(f"Échec du Ping : {packetHandler.getTxRxResult(comm_result)}")
    portHandler.closePort()
    quit()

print(f"Succès ! Servo ST3215 détecté (ID: {SERVO_ID}, Modèle: {model_number})")

# 2. Activation du couple, réglage de l'accélération et de la vitesse
packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 1)
packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_ACCELERATION, 50)
packetHandler.write2ByteTxRx(portHandler, SERVO_ID, ADDR_GOAL_SPEED, 1000)

# 3. Déplacement vers 1800 puis retour au centre (2048)
print("Déplacement vers la position 1800...")
packetHandler.write2ByteTxRx(portHandler, SERVO_ID, ADDR_GOAL_POSITION, 1800)
time.sleep(1.5)

print("Retour au centre (2048)...")
packetHandler.write2ByteTxRx(portHandler, SERVO_ID, ADDR_GOAL_POSITION, 2048)
time.sleep(1.5)

portHandler.closePort()